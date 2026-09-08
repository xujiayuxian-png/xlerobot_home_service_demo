import { act, cleanup, fireEvent, render, screen } from '@testing-library/react'
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { api, type HeadCalibrationState } from './api'
import { HeadCalibrationWorkspace } from './HeadCalibrationWorkspace'

let state: HeadCalibrationState
const props = { unitId: 'robot-1', onError: vi.fn() }
const flush = async () => { await act(async () => {}) }
const advance = async (ms = 500) => { await act(async () => { await vi.advanceTimersByTimeAsync(ms) }) }
const button = (name: string) => screen.getByRole('button', { name }) as HTMLButtonElement
beforeEach(() => {
  vi.useFakeTimers()
  props.onError.mockClear()
  state = {
    available: true, state_fresh: true, state_age_s: .1, action_ready: true,
    request_inflight: false, unit_id: 'robot-1', running: false, phase: 'IDLE',
    message: '标定服务准备就绪', pose_index: -1, pose_count: 25, sample_count: 0,
    target_sample_count: 12, pose_states: Array(25).fill('pending'),
    pose_pan: Array(25).fill(0), pose_tilt: Array(25).fill(.8),
    result_uri: '', quality_passed: false, metrics: {},
    target: { accepted: true, fresh: true, age_s: .1, tag_count: 16, reprojection_rmse_px: .35 },
  }
  vi.spyOn(api, 'headCalibrationStatus').mockImplementation(async () => structuredClone(state))
  vi.spyOn(api, 'startHeadCalibration').mockResolvedValue({ accepted: true, message: '已启动' })
  vi.spyOn(api, 'pauseHeadCalibration').mockResolvedValue({ message: '已请求暂停' })
  vi.spyOn(api, 'moveCalibrationPose').mockResolvedValue({ pose_index: 0, pose_count: 25, pose_name: '中心', message: '已到位' })
})
afterEach(() => { cleanup(); vi.useRealTimers(); vi.restoreAllMocks() })

describe('head camera automatic calibration', () => {
  it('mounts as a read-only observer with dynamic poses and real debug stream', async () => {
    render(<HeadCalibrationWorkspace {...props} />)
    await flush()
    expect(api.startHeadCalibration).not.toHaveBeenCalled()
    expect(api.moveCalibrationPose).not.toHaveBeenCalled()
    expect(screen.getAllByRole('listitem')).toHaveLength(25)
    expect(screen.getByRole('img').getAttribute('src')).toBe('/api/v1/cameras/detection/stream')
    expect(screen.getByText('整板识别通过')).toBeTruthy()
    expect(button('开始自动标定').disabled).toBe(true)
    expect(button('头部低头到预览位（0 / 0.8 rad）').disabled).toBe(true)
  })

  it('requires explicit confirmation and starts only one server-owned sequence', async () => {
    render(<HeadCalibrationWorkspace {...props} />)
    await flush()
    fireEvent.click(screen.getByRole('checkbox'))
    fireEvent.click(button('开始自动标定'))
    await flush()
    expect(api.startHeadCalibration).toHaveBeenCalledExactlyOnceWith('robot-1')
    expect(button('自动采集中…').disabled).toBe(true)
    state = { ...state, running: true, phase: 'MOVING', request_inflight: true, pose_index: 0 }
    await advance(3000)
    expect(api.startHeadCalibration).toHaveBeenCalledOnce()
    expect(api.moveCalibrationPose).not.toHaveBeenCalled()
    expect(button('头部低头到预览位（0 / 0.8 rad）').disabled).toBe(true)
  })

  it('refreshes a running session without launching motion and pauses on the ROS server', async () => {
    state = { ...state, running: true, phase: 'WAITING', sample_count: 6 }
    const first = render(<HeadCalibrationWorkspace {...props} />)
    await flush()
    first.unmount()
    render(<HeadCalibrationWorkspace {...props} />)
    await flush()
    expect(api.startHeadCalibration).not.toHaveBeenCalled()
    fireEvent.click(button('暂停并保留样本'))
    await flush()
    expect(api.pauseHeadCalibration).toHaveBeenCalledOnce()
    state = { ...state, running: false, phase: 'PAUSED' }
    await advance()
    expect(button('继续自动采集 / 补采').disabled).toBe(true)
    fireEvent.click(screen.getByRole('checkbox'))
    fireEvent.click(button('继续自动采集 / 补采'))
    await flush()
    expect(api.startHeadCalibration).toHaveBeenCalledExactlyOnceWith('robot-1')
    expect(screen.getByText('6')).toBeTruthy()
  })

  it('blocks start on stale or missing node but keeps pause available after connection loss', async () => {
    state.state_fresh = false
    render(<HeadCalibrationWorkspace {...props} />)
    await flush()
    fireEvent.click(screen.getByRole('checkbox'))
    expect(button('开始自动标定').disabled).toBe(true)
    state = { ...state, state_fresh: true, running: true, phase: 'MOVING' }
    await advance()
    vi.mocked(api.headCalibrationStatus).mockRejectedValue(new Error('offline'))
    await advance()
    expect(button('暂停并保留样本').disabled).toBe(false)
    fireEvent.click(button('暂停并保留样本'))
    await flush()
    expect(api.pauseHeadCalibration).toHaveBeenCalledOnce()
  })

  it('does not show old accepted target as live after stale status or stalled HTTP', async () => {
    render(<HeadCalibrationWorkspace {...props} />)
    await flush()
    vi.mocked(api.headCalibrationStatus).mockImplementation(() => new Promise(() => {}))
    await advance(5000)
    expect(screen.queryByText('整板识别通过')).toBeNull()
    fireEvent.click(screen.getByRole('checkbox'))
    expect(button('开始自动标定').disabled).toBe(true)
    // The next polling request is scheduled only after the in-flight read ends.
    expect(api.headCalibrationStatus).toHaveBeenCalledTimes(2)
  })

  it('shows quality and result path without any automatic activation', async () => {
    const activate = vi.spyOn(api, 'activateCalibration')
    state = { ...state, phase: 'COMPLETED', quality_passed: true, sample_count: 15,
      pose_states: [...Array(15).fill('captured'), ...Array(10).fill('skipped')],
      result_uri: 'file:///unit/draft/head-camera.yaml', metrics: { translation_rmse_mm: 2.4 } }
    render(<HeadCalibrationWorkspace {...props} />)
    await flush()
    expect(screen.getByText('质量检查通过，结果已保存')).toBeTruthy()
    expect(screen.getByText('file:///unit/draft/head-camera.yaml')).toBeTruthy()
    expect(screen.getByText('2.400')).toBeTruthy()
    expect(button('本次标定已完成').disabled).toBe(true)
    expect(screen.getAllByText('已跳过')).toHaveLength(10)
    expect(screen.queryByText('待补采')).toBeNull()
    expect(activate).not.toHaveBeenCalled()
    expect(screen.getByText('./tools/calibrate capture head-camera --hardware --fresh')).toBeTruthy()
  })

  it('uses the existing single-pose action only for explicitly confirmed preview', async () => {
    render(<HeadCalibrationWorkspace {...props} />)
    await flush()
    fireEvent.click(screen.getByRole('checkbox'))
    fireEvent.click(button('头部低头到预览位（0 / 0.8 rad）'))
    await flush()
    expect(api.moveCalibrationPose).toHaveBeenCalledExactlyOnceWith(0)
    expect(api.startHeadCalibration).not.toHaveBeenCalled()
  })
})
