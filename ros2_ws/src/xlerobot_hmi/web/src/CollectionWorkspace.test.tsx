import { cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react'
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { CollectionWorkspace } from './App'
import { api } from './api'
import type { CollectionState } from './types'

afterEach(() => { cleanup(); vi.restoreAllMocks() })
beforeEach(() => { vi.spyOn(api, 'collectionEpisodes').mockResolvedValue({ episodes: [] }) })
const waiting = {
  dataset_id: 'trial', episode_id: 'episode-1', template_id: 'pick',
  object_id: '羽毛球', dry_run: false, status: 'RUNNING', phase: 'WAITING_HOME',
  progress: .38, message: '', elapsed_s: 0, frame_count: 0,
  episode_uri: '', quality_passed: false,
} as CollectionState

describe('two-stage collection controls', () => {
  it('shows robot storage and updates the displayed path with dataset name', () => {
    render(<CollectionWorkspace state={null} initialDatasetId="trial" onState={vi.fn()} onError={vi.fn()}
      storage={{ root: '/robot/artifacts', config_file: '/robot/config/local.yaml' }} />)
    expect(screen.getByText('/robot/artifacts/datasets/trial')).toBeTruthy()
    expect(screen.getByText('/robot/config/local.yaml')).toBeTruthy()
    fireEvent.change(screen.getByLabelText('数据集名称'), { target: { value: 'new-trial' } })
    expect(screen.getByText('/robot/artifacts/datasets/new-trial')).toBeTruthy()
  })

  it('shows a prominent initial pose warning with the specific joint and recovery steps', () => {
    const health = { diagnostics: { 'xlerobot/collection_initial_pose': {
      level: 2, message: '主臂 leader_gripper：当前 -0.100 rad，允许 [0.000, 1.650] rad',
    } } } as unknown as import('./types').Health
    render(<CollectionWorkspace state={null} initialDatasetId="trial" health={health} onState={vi.fn()} onError={vi.fn()} />)
    expect(screen.getByRole('alert').textContent).toContain('初始姿态不合适')
    expect(screen.getByRole('alert').textContent).toContain('leader_gripper')
    expect(screen.getByRole('alert').textContent).toContain('Reset')
  })

  it('shows default keep with only reject, and can restore a rejection', async () => {
    const review = vi.spyOn(api, 'reviewEpisode').mockResolvedValue({ review_uri: 'file:///review.json' })
    const onState = vi.fn()
    const props = { initialDatasetId: 'trial', onState, onError: vi.fn() }
    const complete: CollectionState = { ...waiting, status: 'SUCCEEDED', phase: 'REVIEW', episode_uri: 'file:///episode', review_status: 'accepted' }
    const view = render(<CollectionWorkspace {...props} state={complete} />)
    expect(screen.getByText('本条：已保留')).toBeTruthy()
    expect(screen.queryByRole('button', { name: '接受' })).toBeNull()
    expect(review).not.toHaveBeenCalled()
    fireEvent.click(screen.getByRole('button', { name: '拒绝本条' }))
    await waitFor(() => expect(review).toHaveBeenLastCalledWith('trial', 'episode-1', 'rejected'))
    await waitFor(() => expect(onState).toHaveBeenLastCalledWith({ ...complete, review_status: 'rejected' }))
    expect(screen.getByText(/已拒绝，后续转换将跳过/)).toBeTruthy()
    view.rerender(<CollectionWorkspace {...props} state={{ ...complete, review_status: 'rejected' }} />)
    fireEvent.click(screen.getByRole('button', { name: '恢复保留本条' }))
    await waitFor(() => expect(review).toHaveBeenLastCalledWith('trial', 'episode-1', 'accepted'))
  })

  it('can reject a previous episode while recording another without changing current state', async () => {
    vi.mocked(api.collectionEpisodes).mockResolvedValue({ episodes: [{
      dataset_id: 'trial', episode_id: 'older-episode', created_at: '',
      frame_count: 30, duration_s: 1, review_status: 'accepted',
    }] })
    const review = vi.spyOn(api, 'reviewEpisode').mockResolvedValue({ review_uri: 'file:///review.json' })
    const onState = vi.fn()
    render(<CollectionWorkspace state={{ ...waiting, phase: 'RECORDING' }} initialDatasetId="trial" onState={onState} onError={vi.fn()} />)
    fireEvent.click(await screen.findByRole('button', { name: 'older-episode 拒绝' }))
    await waitFor(() => expect(review).toHaveBeenCalledWith('trial', 'older-episode', 'rejected'))
    await waitFor(() => expect(screen.getByText(/older-episode：已拒绝/)).toBeTruthy())
    expect(onState).not.toHaveBeenCalled()
  })

  it('requires Reset after release and never starts preparation from Reset', async () => {
    const recover = vi.spyOn(api, 'recoverCollection').mockResolvedValue({
      message: 'torque off', collection: null,
    })
    const start = vi.spyOn(api, 'startCollection')
    render(<CollectionWorkspace state={null} initialDatasetId="trial" onState={vi.fn()} onError={vi.fn()} />)
    fireEvent.click(screen.getByRole('button', { name: '释放主从臂扭矩' }))
    await waitFor(() => expect(screen.getByRole('status').textContent).toBe('torque off'))
    expect((screen.getByRole('button', { name: '开始 / 准备 pregrasp' }) as HTMLButtonElement).disabled).toBe(true)
    fireEvent.click(screen.getByRole('button', { name: 'Reset / 重置状态' }))
    await waitFor(() => expect(recover).toHaveBeenLastCalledWith('reset'))
    await waitFor(() => expect((screen.getByRole('button', { name: '开始 / 准备 pregrasp' }) as HTMLButtonElement).disabled).toBe(false))
    expect(start).not.toHaveBeenCalled()
  })

  it('updates instruction when object changes and still allows editing it', () => {
    render(<CollectionWorkspace state={null} initialDatasetId="trial" onState={vi.fn()} onError={vi.fn()} />)
    fireEvent.change(screen.getByPlaceholderText('物体标签'), { target: { value: '黄色胶棒' } })
    expect((screen.getByLabelText('语言指令') as HTMLInputElement).value).toBe('抓住黄色胶棒')
    fireEvent.change(screen.getByLabelText('语言指令'), { target: { value: '拿起黄色胶棒' } })
    expect((screen.getByLabelText('语言指令') as HTMLInputElement).value).toBe('拿起黄色胶棒')
  })

  it('Start only submits preparation, Home is separate and waits for readiness', async () => {
    const start = vi.spyOn(api, 'startCollection').mockResolvedValue(waiting)
    const home = vi.spyOn(api, 'beginCollection').mockResolvedValue(waiting)
    const props = { initialDatasetId: 'trial', onState: vi.fn(), onError: vi.fn() }
    const view = render(<CollectionWorkspace {...props} state={null} />)
    fireEvent.click(screen.getByRole('button', { name: '开始 / 准备 pregrasp' }))
    await waitFor(() => expect(start).toHaveBeenCalledTimes(1))
    expect(home).not.toHaveBeenCalled()
    view.rerender(<CollectionWorkspace {...props} state={{ ...waiting, phase: 'PREPARE_PREGRASP' }} />)
    expect((screen.getByRole('button', { name: 'Home / 释放主臂并开始采集' }) as HTMLButtonElement).disabled).toBe(true)
    view.rerender(<CollectionWorkspace {...props} state={waiting} />)
    expect((screen.getByRole('button', { name: 'End / 结束并保存到本机' }) as HTMLButtonElement).disabled).toBe(true)
    fireEvent.click(screen.getByRole('button', { name: 'Home / 释放主臂并开始采集' }))
    await waitFor(() => expect(home).toHaveBeenCalledWith('trial', 'episode-1'))
  })

  it('gates keyboard Home/End by phase and ignores typing and repeat', async () => {
    const home = vi.spyOn(api, 'beginCollection').mockResolvedValue(waiting)
    const end = vi.spyOn(api, 'finalizeCollection').mockResolvedValue(waiting)
    const props = { initialDatasetId: 'trial', onState: vi.fn(), onError: vi.fn() }
    const view = render(<CollectionWorkspace {...props} state={waiting} />)
    fireEvent.keyDown(screen.getByLabelText('语言指令'), { key: 'Home' })
    fireEvent.keyDown(window, { key: 'Home', repeat: true })
    expect(home).not.toHaveBeenCalled()
    fireEvent.keyDown(window, { key: 'Home' })
    await waitFor(() => expect(home).toHaveBeenCalledTimes(1))
    view.rerender(<CollectionWorkspace {...props} state={{ ...waiting, phase: 'STARTING_RECORDING' }} />)
    fireEvent.keyDown(window, { key: 'Home' })
    fireEvent.keyDown(window, { key: 'End' })
    expect(home).toHaveBeenCalledTimes(1)
    expect(end).not.toHaveBeenCalled()
    view.rerender(<CollectionWorkspace {...props} state={{ ...waiting, phase: 'RECORDING' }} />)
    fireEvent.keyDown(window, { key: 'End' })
    await waitFor(() => expect(end).toHaveBeenCalledTimes(1))
  })
})
