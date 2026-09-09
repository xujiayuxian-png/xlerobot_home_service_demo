import { act, cleanup, fireEvent, render, screen, within } from '@testing-library/react'
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { api, type ServoCalibrationCommand, type ServoCalibrationState } from './api'
import { ServoCalibrationWorkspace } from './ServoCalibrationWorkspace'

const allGroups = ['right_arm', 'left_arm', 'head']
const armNames = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
function initialState(): ServoCalibrationState {
  const joints = allGroups.flatMap(group =>
    (group === 'head' ? ['pan', 'tilt'] : armNames).map((name, index) => ({
      name: `${group}.${name}`, servo_id: index + 1, position: 2000, zero: -1,
      raw_min: -1, raw_max: -1, coverage: 0, zero_captured: false,
      reference_zero: -1, zero_source: '', range_captured: false, online: true, message: '',
    })))
  return { phase: 'IDLE', joint_names: joints.map(joint => joint.name),
    raw_positions: joints.map(joint => joint.position), result_uri: '', active_group: '',
    completed_groups: [], released_groups: [], session_uri: 'file:///unit/servo-session.json', joints }
}
let server: ServoCalibrationState
const props = { unitId: 'robot-1', captureOnly: true, onError: vi.fn() }
const flush = async () => { await act(async () => {}) }
const advance = async (ms = 500) => { await act(async () => { await vi.advanceTimersByTimeAsync(ms) }) }
const button = (name: string) => screen.getByRole('button', { name }) as HTMLButtonElement
const applyCommand = (command: ServoCalibrationCommand, group: string) => {
  if (command === 'release_torque') server = { ...server, released_groups: [...server.released_groups, group] }
  if (command === 'capture_zero' || command === 'use_existing_zero') server = {
    ...server, active_group: group, phase: 'ZERO_CAPTURED',
    joints: server.joints.map(joint => joint.name.startsWith(`${group}.`) ? {
      ...joint, zero: command === 'capture_zero' ? joint.position : joint.reference_zero,
      zero_captured: true, zero_source: command === 'capture_zero' ? 'measured' : 'existing:verified-version',
    } : joint),
  }
  if (command === 'start_range') server = { ...server, active_group: group, phase: 'RANGE_RECORDING' }
  if (command === 'pause_range') server = { ...server, phase: 'PAUSED' }
  if (command === 'finish_range') server = {
    ...server, phase: 'RANGE_CAPTURED', active_group: '', completed_groups: [...server.completed_groups, group],
  }
  if (command === 'finalize') server = { ...server, phase: 'FINALIZED', result_uri: 'file:///unit/servo-result.yaml' }
  if (command === 'reset_group') server = {
    ...server, completed_groups: server.completed_groups.filter(item => item !== group),
    joints: server.joints.map(joint => joint.name.startsWith(`${group}.`) ? {
      ...joint, zero: -1, raw_min: -1, raw_max: -1, coverage: 0,
      zero_captured: false, range_captured: false, zero_source: '',
    } : joint),
  }
  return server
}
beforeEach(() => {
  vi.useFakeTimers()
  server = initialState()
  props.onError.mockClear()
  vi.spyOn(api, 'servoCalibrationStatus').mockImplementation(async () => structuredClone(server))
  vi.spyOn(api, 'servoCalibrationStep').mockImplementation(async (_, command, group) => applyCommand(command, group))
})
afterEach(() => { cleanup(); vi.useRealTimers(); vi.restoreAllMocks() })

it('offers only Leader and finalizes one independent group', async () => {
  server.joints = server.joints.slice(0, 6).map(j => ({ ...j, name: j.name.replace('right_arm.', 'leader.') }))
  server.completed_groups = ['leader']
  render(<ServoCalibrationWorkspace {...props} />)
  await flush()
  expect(screen.getByText('1 / 1 组完成')).toBeTruthy()
  const tabs = screen.getByRole('group', { name: '标定分组' })
  expect(within(tabs).getAllByRole('button')).toHaveLength(1)
  expect(within(tabs).getByText('Leader 主臂')).toBeTruthy()
  fireEvent.click(button('完成采集并保存结果'))
  await flush()
  expect(api.servoCalibrationStep).toHaveBeenCalledWith('robot-1', 'finalize', 'leader')
})

describe('whole-group servo calibration', () => {
  it.each([false, true])('offers a restart even after finalized (Leader=%s)', async leader => {
    server.phase = 'FINALIZED'
    if (leader) server.joints = server.joints.slice(0, 6).map(j => ({ ...j, name: j.name.replace('right_arm.', 'leader.') }))
    const restart = vi.fn()
    render(<ServoCalibrationWorkspace {...props} onRestart={restart} />)
    await flush()
    const name = leader ? '重新开始 Leader 标定' : '重新开始从臂 / 头部标定'
    expect(button(name).disabled).toBe(false)
    fireEvent.click(button(name))
    expect(restart).toHaveBeenCalledOnce()
    expect(api.servoCalibrationStep).not.toHaveBeenCalled()
  })
  it('reads on mount without any torque or movement command and shows six joints', async () => {
    render(<ServoCalibrationWorkspace {...props} />)
    await flush()
    expect(api.servoCalibrationStatus).toHaveBeenCalledOnce()
    expect(api.servoCalibrationStep).not.toHaveBeenCalled()
    expect(screen.getAllByRole('progressbar')).toHaveLength(6)
    expect(screen.queryByLabelText('当前关节')).toBeNull()
    expect(button('记录零位并开始整组采集').disabled).toBe(true)
    expect(button('完成采集并保存结果').disabled).toBe(true)
    expect(screen.getByText('file:///unit/servo-session.json')).toBeTruthy()
  })

  it('captures zero then starts all joints, finishes the group, and needs no joint selection', async () => {
    render(<ServoCalibrationWorkspace {...props} />)
    await flush()
    fireEvent.click(button('已托住，释放本组扭矩'))
    await flush()
    fireEvent.click(button('记录零位并开始整组采集'))
    await flush()
    expect(vi.mocked(api.servoCalibrationStep).mock.calls).toEqual([
      ['robot-1', 'release_torque', 'right_arm'],
      ['robot-1', 'capture_zero', 'right_arm'],
      ['robot-1', 'start_range', 'right_arm'],
    ])
    expect(button('02 左臂 5 个关节 + 夹爪 待完成').disabled).toBe(true)
    fireEvent.click(button('完成本组'))
    await flush()
    expect(api.servoCalibrationStep).toHaveBeenLastCalledWith('robot-1', 'finish_range', 'right_arm')
    expect(screen.getByText('1 / 3 组完成')).toBeTruthy()
    expect(button('完成采集并保存结果').disabled).toBe(true)
  })

  it('serializes slow reads and keeps the chosen tab and DOM stable across polls and rerenders', async () => {
    let resolve!: (value: ServoCalibrationState) => void
    vi.mocked(api.servoCalibrationStatus).mockImplementationOnce(() => new Promise(done => { resolve = done }))
    const view = render(<ServoCalibrationWorkspace {...props} />)
    await advance(2000)
    expect(api.servoCalibrationStatus).toHaveBeenCalledOnce()
    await act(async () => resolve(server))
    fireEvent.click(button('02 左臂 5 个关节 + 夹爪 待完成'))
    const table = screen.getByRole('table')
    server = { ...server, active_group: 'right_arm' }
    await advance()
    view.rerender(<ServoCalibrationWorkspace {...props} onError={vi.fn()} />)
    await advance()
    expect(button('02 左臂 5 个关节 + 夹爪 待完成').getAttribute('aria-pressed')).toBe('true')
    expect(screen.getByRole('table')).toBe(table)
    expect(api.servoCalibrationStep).not.toHaveBeenCalled()
    expect(api.servoCalibrationStatus).toHaveBeenCalledTimes(3)
  })

  it('restores a recording group on refresh without restarting it and retains measured ranges', async () => {
    applyCommand('release_torque', 'head')
    applyCommand('capture_zero', 'head')
    applyCommand('start_range', 'head')
    server = { ...server, joints: server.joints.map(joint => ({ ...joint, raw_min: 1000, raw_max: 3000, coverage: .75 })) }
    const view = render(<ServoCalibrationWorkspace {...props} />)
    await flush()
    expect(button('03 头部 水平转动 + 俯仰 采集中').getAttribute('aria-pressed')).toBe('true')
    expect(screen.getAllByRole('progressbar')).toHaveLength(2)
    view.unmount()
    render(<ServoCalibrationWorkspace {...props} />)
    await flush()
    expect(button('完成本组')).toBeTruthy()
    expect(screen.getAllByText('1000 → 3000')).toHaveLength(2)
    expect(api.servoCalibrationStep).not.toHaveBeenCalled()
  })

  it('prevents repeated clicks during a write and ignores a stale poll arriving after it', async () => {
    render(<ServoCalibrationWorkspace {...props} />)
    await flush()
    const oldState = server
    let resolveRead!: (value: ServoCalibrationState) => void
    vi.mocked(api.servoCalibrationStatus).mockImplementationOnce(() => new Promise(done => { resolveRead = done }))
    await advance()
    let resolveWrite!: (value: ServoCalibrationState) => void
    vi.mocked(api.servoCalibrationStep).mockImplementationOnce(() => new Promise(done => { resolveWrite = done }))
    const release = button('已托住，释放本组扭矩')
    fireEvent.click(release)
    fireEvent.click(release)
    expect(api.servoCalibrationStep).toHaveBeenCalledOnce()
    expect(release.disabled).toBe(true)
    expect(screen.getByRole('status').textContent).toBe('正在释放本组扭矩…')
    await act(async () => resolveWrite(applyCommand('release_torque', 'right_arm')))
    await act(async () => resolveRead(oldState))
    expect(button('记录零位并开始整组采集').disabled).toBe(false)
  })

  it('preserves recording and data when finish reports an insufficient range', async () => {
    applyCommand('release_torque', 'right_arm')
    applyCommand('capture_zero', 'right_arm')
    applyCommand('start_range', 'right_arm')
    vi.mocked(api.servoCalibrationStep).mockRejectedValueOnce(new Error('wrist_roll coverage 0.20 < 0.60'))
    render(<ServoCalibrationWorkspace {...props} />)
    await flush()
    fireEvent.click(button('完成本组'))
    await flush()
    expect(screen.getByRole('alert').textContent).toContain('范围仍在记录')
    expect(screen.getByRole('alert').textContent).toContain('wrist_roll coverage 0.20')
    await advance(1000)
    expect(screen.getByRole('alert').textContent).toContain('wrist_roll coverage 0.20')
    expect(button('暂停采集').disabled).toBe(false)
    expect(button('完成本组').disabled).toBe(false)
    expect(button('完成采集并保存结果').disabled).toBe(true)
    expect(props.onError).toHaveBeenCalledOnce()
  })

  it('pauses and continues without re-capturing zero or clearing ranges', async () => {
    applyCommand('release_torque', 'right_arm')
    applyCommand('capture_zero', 'right_arm')
    applyCommand('start_range', 'right_arm')
    server = { ...server, joints: server.joints.map(joint => ({ ...joint, raw_min: 1500, raw_max: 2500, coverage: .7 })) }
    render(<ServoCalibrationWorkspace {...props} />)
    await flush()
    fireEvent.click(button('暂停采集'))
    await flush()
    fireEvent.click(button('继续采集（保留已有范围）'))
    await flush()
    expect(vi.mocked(api.servoCalibrationStep).mock.calls.map(call => call[1])).toEqual(['pause_range', 'start_range'])
    expect(screen.getAllByText('1500 → 2500')).toHaveLength(6)
  })

  it('requires explicit confirmation to reset only this group and gates final saving', async () => {
    server = { ...server, completed_groups: [...allGroups] }
    render(<ServoCalibrationWorkspace {...props} />)
    await flush()
    expect(button('完成采集并保存结果').disabled).toBe(false)
    fireEvent.click(button('重新采集这一组…'))
    expect(screen.getByRole('alertdialog').textContent).toContain('其他组、已激活标定和舵机设置不变')
    expect(api.servoCalibrationStep).not.toHaveBeenCalled()
    fireEvent.click(button('取消'))
    expect(screen.queryByRole('alertdialog')).toBeNull()
    fireEvent.click(button('重新采集这一组…'))
    fireEvent.click(button('确认重置本组'))
    await flush()
    expect(api.servoCalibrationStep).toHaveBeenCalledWith('robot-1', 'reset_group', 'right_arm')
    expect(server.completed_groups).toEqual(['left_arm', 'head'])
    expect(button('完成采集并保存结果').disabled).toBe(true)
  })

  it('displays the actual result path after all groups are saved without re-enabling torque', async () => {
    server = { ...server, completed_groups: [...allGroups] }
    render(<ServoCalibrationWorkspace {...props} />)
    await flush()
    fireEvent.click(button('完成采集并保存结果'))
    await flush()
    expect(screen.getByText('file:///unit/servo-result.yaml')).toBeTruthy()
    expect(button('标定结果已保存').disabled).toBe(true)
    expect(api.servoCalibrationStep).toHaveBeenCalledOnce()
    expect(api.servoCalibrationStep).toHaveBeenCalledWith('robot-1', 'finalize', 'right_arm')
  })

  it('does not chain range recording if zero capture fails', async () => {
    applyCommand('release_torque', 'right_arm')
    vi.mocked(api.servoCalibrationStep).mockRejectedValueOnce(new Error('zero capture failed'))
    render(<ServoCalibrationWorkspace {...props} />)
    await flush()
    fireEvent.click(button('记录零位并开始整组采集'))
    await flush()
    expect(api.servoCalibrationStep).toHaveBeenCalledOnce()
    expect(api.servoCalibrationStep).toHaveBeenCalledWith('robot-1', 'capture_zero', 'right_arm')
    expect(screen.getByRole('alert').textContent).toContain('zero capture failed')
  })

  it('requires both explicit release and unchanged-hardware confirmation to reuse existing zero', async () => {
    server = { ...server, joints: server.joints.map(joint => ({ ...joint, reference_zero: 2048 })) }
    render(<ServoCalibrationWorkspace {...props} />)
    await flush()
    fireEvent.click(screen.getByText('未拆装过？也可以保留已有标定零位'))
    expect(button('保留已有零位并开始采集').disabled).toBe(true)
    fireEvent.click(screen.getByLabelText('未拆装舵机 / 未更改硬件零偏'))
    expect(button('保留已有零位并开始采集').disabled).toBe(true)
    fireEvent.click(button('已托住，释放本组扭矩'))
    await flush()
    expect(button('保留已有零位并开始采集').disabled).toBe(false)
    fireEvent.click(button('保留已有零位并开始采集'))
    await flush()
    expect(vi.mocked(api.servoCalibrationStep).mock.calls.map(call => call[1])).toEqual([
      'release_torque', 'use_existing_zero', 'start_range',
    ])
    expect(screen.getByText(/来源：沿用已有有效标定/)).toBeTruthy()
  })

  it('shows unavailable raw values as dashes, exposes wrap/read errors, and recovers read polling', async () => {
    server.joints[0] = { ...server.joints[0], position: -1, online: false, message: 'encoder wrap: pause and reposition' }
    render(<ServoCalibrationWorkspace {...props} />)
    await flush()
    const row = screen.getByRole('rowheader', { name: /肩部水平/ }).closest('tr')!
    expect(within(row).getAllByText('—').length).toBeGreaterThan(0)
    // A hardware-specific message must not be lost behind a generic offline label.
    expect(row.textContent).toContain('encoder wrap')
    expect(button('记录零位并开始整组采集').disabled).toBe(true)
    vi.mocked(api.servoCalibrationStatus).mockRejectedValueOnce(new Error('network read failed'))
    await advance()
    expect(screen.getByRole('alert').textContent).toContain('最后一次读数')
    expect(button('已托住，释放本组扭矩').disabled).toBe(true)
    await advance()
    expect(screen.queryByRole('alert')).toBeNull()
    expect(button('已托住，释放本组扭矩').disabled).toBe(false)
  })

  it('translates healthy backend messages without warning color and checks zero containment and faults', async () => {
    applyCommand('release_torque', 'right_arm')
    applyCommand('capture_zero', 'right_arm')
    applyCommand('start_range', 'right_arm')
    server = { ...server, joints: server.joints.map(joint => ({
      ...joint, raw_min: 1000, raw_max: 3000, coverage: .9, message: 'range sufficient',
    })) }
    render(<ServoCalibrationWorkspace {...props} />)
    await flush()
    const row = screen.getByRole('rowheader', { name: /肩部水平/ }).closest('tr')!
    expect(row.textContent).toContain('范围已达标')
    expect(row.classList.contains('servo-joint-warning')).toBe(false)
    expect(screen.getByText('所有关节范围已达标，可以完成本组。')).toBeTruthy()
    server.joints[0] = { ...server.joints[0], zero: 500, message: 'include the zero pose in the recorded range' }
    await advance()
    expect(screen.queryByText('所有关节范围已达标，可以完成本组。')).toBeNull()
    expect(row.textContent).toContain('范围还未覆盖零位')
    expect(row.classList.contains('servo-joint-warning')).toBe(true)
    expect(within(row).getByRole('progressbar').classList.contains('enough')).toBe(false)
    server.joints[0] = { ...server.joints[0], zero: 2000,
      message: 'encoder wrapped or jumped >2048 ticks; reset this group; do not cross encoder zero' }
    await advance()
    expect(row.textContent).toContain('编码器跨零 / 跳变')
    expect(screen.queryByText('所有关节范围已达标，可以完成本组。')).toBeNull()
  })
})

describe('servo HTTP contract', () => {
  it('sends one group and no joint selector', async () => {
    vi.mocked(api.servoCalibrationStep).mockRestore()
    const fetchMock = vi.spyOn(globalThis, 'fetch').mockResolvedValue({ ok: true, json: async () => server } as Response)
    await api.servoCalibrationStep('robot-1', 'start_range', 'right_arm')
    expect(fetchMock).toHaveBeenCalledWith('/api/v1/calibrations/servo/step', expect.objectContaining({
      method: 'POST', body: JSON.stringify({ unit_id: 'robot-1', command: 'start_range', group: 'right_arm' }),
    }))
  })
})
