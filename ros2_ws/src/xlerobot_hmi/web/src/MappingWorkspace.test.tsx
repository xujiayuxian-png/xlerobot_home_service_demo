import { act, cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react'
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { JoystickPad as BaseJoystick, joystickVector } from './JoystickPad'
import { BaseJoystick as OperatorJoystick, MappingWorkspace } from './App'
import { api, type SiteSummary } from './api'
import type { MappingState } from './types'

class TestPointerEvent extends MouseEvent {
  pointerId: number
  constructor(type: string, options: PointerEventInit = {}) {
    super(type, options)
    this.pointerId = options.pointerId ?? 1
  }
}

class TestSocket {
  static OPEN = 1
  static CLOSING = 2
  static instances: TestSocket[] = []
  readyState = 0
  sent: Array<{ armed: boolean, linear: number, angular: number }> = []
  onopen = () => {}
  onclose = () => {}
  onerror = () => {}
  onmessage = (_event: { data: string }) => {}
  constructor() { TestSocket.instances.push(this) }
  send(value: string) { this.sent.push(JSON.parse(value)) }
  close() { this.readyState = 3 }
  open() { this.readyState = 1; this.onopen() }
}

const summary: SiteSummary = {
  site_id: 'home', active_version: '', draft: {
    map_saved: true, map_name: 'ground-floor', map_saved_at: '2026-09-05T09:42:00Z',
    places: [{ id: 'table', x: .1, y: -.07, yaw: 2, nav_offset_m: .25,
      dock: true, validated: false }], ready: false,
  },
}
const state: MappingState = { slam: 'MAPPING', map: null,
  pose: { frame_id: 'map', x: .1, y: -.07, yaw: 2 }, scan: null, path: null }

beforeEach(() => {
  vi.stubGlobal('PointerEvent', TestPointerEvent)
  vi.stubGlobal('WebSocket', TestSocket)
  TestSocket.instances = []
  vi.spyOn(api, 'site').mockResolvedValue(structuredClone(summary))
  vi.spyOn(HTMLElement.prototype, 'getBoundingClientRect').mockReturnValue({
    x: 0, y: 0, top: 0, left: 0, right: 212, bottom: 212, width: 212, height: 212,
    toJSON: () => ({}),
  })
  HTMLElement.prototype.setPointerCapture = vi.fn()
  HTMLElement.prototype.releasePointerCapture = vi.fn()
  HTMLElement.prototype.hasPointerCapture = vi.fn(() => true)
})
afterEach(() => { cleanup(); vi.restoreAllMocks(); vi.unstubAllGlobals(); vi.useRealTimers() })

const press = (element: HTMLElement, x = 106, y = 28, id = 1) =>
  fireEvent.pointerDown(element, { pointerId: id, button: 0, buttons: 1, clientX: x, clientY: y })

describe('mapping joystick input', () => {
  it('has a center dead zone, correct ROS axes, and bounded diagonal output', () => {
    expect(joystickVector(.05, .05).linear).toBe(0)
    expect(joystickVector(0, -1).linear).toBe(1)
    expect(joystickVector(1, 0).angular).toBe(-1)
    const diagonal = joystickVector(10, -10)
    expect(Math.hypot(diagonal.linear, diagonal.angular)).toBeCloseTo(1)
  })
  it('captures one pointer, prevents selection, supports drag-out, and releases once', () => {
    const onMove = vi.fn(), onRelease = vi.fn()
    render(<BaseJoystick disabled={false} onMove={onMove} onRelease={onRelease} />)
    const pad = screen.getByRole('group', { name: '底盘摇杆' })
    expect(press(pad)).toBe(false) // pointerdown default prevented
    expect(onMove).toHaveBeenLastCalledWith(1, -0)
    press(pad, 20, 20, 2)
    fireEvent.pointerUp(pad, { pointerId: 2 })
    expect(onRelease).not.toHaveBeenCalled()
    fireEvent.pointerLeave(pad, { pointerId: 1 })
    expect(onRelease).not.toHaveBeenCalled()
    fireEvent.pointerMove(pad, { pointerId: 1, buttons: 1, clientX: 600, clientY: -300 })
    expect(Math.hypot(...onMove.mock.lastCall!)).toBeCloseTo(1)
    fireEvent.pointerUp(pad, { pointerId: 1 })
    fireEvent.lostPointerCapture(pad, { pointerId: 1 })
    expect(onRelease).toHaveBeenCalledTimes(1)
  })
  it.each(['pointerCancel', 'lostPointerCapture', 'blur', 'hidden', 'unmount', 'disabled', 'buttonsLost'])(
    'zeros the input on %s', event => {
      const onMove = vi.fn(), onRelease = vi.fn()
      const view = render(<BaseJoystick disabled={false} onMove={onMove} onRelease={onRelease} />)
      const pad = screen.getByRole('group', { name: '底盘摇杆' })
      press(pad)
      if (event === 'blur') fireEvent.blur(window)
      else if (event === 'hidden') {
        vi.spyOn(document, 'hidden', 'get').mockReturnValue(true)
        fireEvent(document, new Event('visibilitychange'))
      } else if (event === 'unmount') view.unmount()
      else if (event === 'disabled') view.rerender(<BaseJoystick disabled onMove={onMove} onRelease={onRelease} />)
      else if (event === 'buttonsLost') fireEvent.pointerMove(pad, { pointerId: 1, buttons: 0 })
      else fireEvent[event as 'pointerCancel' | 'lostPointerCapture'](pad, { pointerId: 1 })
      expect(onRelease).toHaveBeenCalledTimes(1)
    },
  )
  it('does not drive while disabled or on a secondary mouse button', () => {
    const move = vi.fn()
    const view = render(<BaseJoystick disabled onMove={move} onRelease={vi.fn()} />)
    const pad = screen.getByRole('group', { name: '底盘摇杆' })
    press(pad)
    view.rerender(<BaseJoystick disabled={false} onMove={move} onRelease={vi.fn()} />)
    fireEvent.pointerDown(pad, { pointerId: 1, button: 2 })
    expect(move).not.toHaveBeenCalled()
  })
})

describe('mapping workflow', () => {
  it('keeps one held command through parent updates and the five-second site refresh', async () => {
    vi.useFakeTimers()
    const view = render(<MappingWorkspace state={state} phase="build" initialSiteId="home" onError={vi.fn()} />)
    await act(async () => {})
    fireEvent.click(screen.getByText('开启遥控'))
    const pad = screen.getByRole('group', { name: '底盘摇杆' })
    press(pad)
    const socket = TestSocket.instances[0]
    act(() => socket.open())
    for (let i = 0; i < 120; i++) {
      view.rerender(<MappingWorkspace state={{ ...state, pose: { ...state.pose!, x: i / 100 } }}
        phase="build" initialSiteId="home" onError={vi.fn()} />)
      await act(async () => { vi.advanceTimersByTime(100) })
    }
    expect(api.site).toHaveBeenCalledTimes(3)
    expect(TestSocket.instances).toHaveLength(1)
    expect(socket.sent).toHaveLength(121)
    expect(socket.sent.every(command => command.armed && command.linear === .08)).toBe(true)
    expect(screen.getByRole('group', { name: '底盘摇杆' })).toBe(pad)
  })
  it('blocks duplicate requests while saving and reports failure without claiming success', async () => {
    let reject!: (error: Error) => void
    const save = vi.spyOn(api, 'setPlace').mockImplementation(() => new Promise((_, fail) => { reject = fail }))
    const error = vi.fn()
    render(<MappingWorkspace state={state} phase="build" initialSiteId="home" onError={error} />)
    await screen.findByRole('button', { name: '用当前位置更新地点' })
    fireEvent.change(screen.getByLabelText('地点 ID'), { target: { value: 'home' } })
    const button = screen.getByRole('button', { name: '记录当前位置为地点' })
    fireEvent.click(button)
    fireEvent.click(button)
    expect(save).toHaveBeenCalledTimes(1)
    expect((button as HTMLButtonElement).disabled).toBe(true)
    expect(screen.getByRole('status').textContent).toBe('记录地点中…')
    await act(async () => reject(new Error('pose unavailable')))
    expect(error).toHaveBeenCalledWith('Error: pose unavailable')
    expect(screen.getByRole('status').textContent).toBe('')
    expect((button as HTMLButtonElement).disabled).toBe(false)
  })
  it('loads persisted places, prevents saving while armed, and confirms replacement', async () => {
    const save = vi.spyOn(api, 'setPlace').mockResolvedValue({ artifact_uri: 'draft' })
    const confirm = vi.spyOn(window, 'confirm').mockReturnValue(false)
    render(<MappingWorkspace state={state} phase="build" initialSiteId="home" onError={vi.fn()} />)
    const update = await screen.findByRole('button', { name: '用当前位置更新地点' })
    expect(screen.getByRole('list', { name: '已保存地点' }).textContent).toContain('table')
    fireEvent.click(screen.getByText('开启遥控'))
    expect((update as HTMLButtonElement).disabled).toBe(true)
    fireEvent.click(screen.getByText('停车并结束遥控'))
    fireEvent.click(update)
    expect(confirm).toHaveBeenCalled()
    expect(save).not.toHaveBeenCalled()
    confirm.mockReturnValue(true)
    fireEvent.click(update)
    await waitFor(() => expect(save).toHaveBeenCalledWith('home', 'table', true, .25))
  })
  it('reuses the connection on release/re-press, sends zero heartbeats, and disarms on blur', async () => {
    vi.useFakeTimers()
    render(<MappingWorkspace state={state} phase="build" initialSiteId="home" onError={vi.fn()} />)
    await act(async () => {})
    fireEvent.click(screen.getByText('开启遥控'))
    const pad = screen.getByRole('group', { name: '底盘摇杆' })
    press(pad)
    const socket = TestSocket.instances[0]
    act(() => socket.open())
    expect(socket.sent.at(-1)?.linear).toBe(.08)
    fireEvent.pointerUp(pad, { pointerId: 1 })
    act(() => { vi.advanceTimersByTime(200) })
    expect(socket.sent.at(-1)).toEqual({ armed: true, linear: 0, angular: 0 })
    press(pad)
    expect(TestSocket.instances).toHaveLength(1)
    fireEvent.blur(window)
    expect(socket.sent.at(-1)).toEqual({ armed: false, linear: 0, angular: 0 })
    expect(screen.getByText('开启遥控')).toBeTruthy()
  })
  it('does not send delayed motion if the stick is released before connection opens', async () => {
    render(<MappingWorkspace state={state} phase="build" initialSiteId="home" onError={vi.fn()} />)
    await screen.findByText('用当前位置更新地点')
    fireEvent.click(screen.getByText('开启遥控'))
    const pad = screen.getByRole('group', { name: '底盘摇杆' })
    press(pad)
    fireEvent.pointerUp(pad, { pointerId: 1 })
    act(() => TestSocket.instances[0].open())
    expect(TestSocket.instances[0].sent).toEqual([{ armed: true, linear: 0, angular: 0 }])
  })
  it('ignores stale socket callbacks and disarms on a current disconnect', async () => {
    const error = vi.fn()
    render(<MappingWorkspace state={state} phase="build" initialSiteId="home" onError={error} />)
    await screen.findByText('用当前位置更新地点')
    fireEvent.click(screen.getByText('开启遥控'))
    const pad = screen.getByRole('group', { name: '底盘摇杆' })
    press(pad)
    const old = TestSocket.instances[0]
    fireEvent.click(screen.getByText('停车并结束遥控'))
    fireEvent.click(screen.getByText('开启遥控'))
    press(pad)
    const current = TestSocket.instances[1]
    act(() => { old.open(); old.onerror(); old.onclose(); old.onmessage({ data: '{"error":"old"}' }) })
    expect(error).not.toHaveBeenCalled()
    expect(current.readyState).toBe(0)
    act(() => { current.open(); current.onclose() })
    expect(error).toHaveBeenCalledTimes(1)
    expect(screen.getByText('开启遥控')).toBeTruthy()
  })
  it('requires localization before navigating, and complete validation before activation', async () => {
    render(<MappingWorkspace state={state} phase="validate" initialSiteId="home" onError={vi.fn()} />)
    await waitFor(() => expect((screen.getByText('1. 自动定位验证') as HTMLButtonElement).disabled).toBe(false))
    expect((screen.getByText('2. 导航并验证地点') as HTMLButtonElement).disabled).toBe(true)
    expect((screen.getByText('3. 激活场地草稿') as HTMLButtonElement).disabled).toBe(true)
    expect(TestSocket.instances).toHaveLength(0)
  })
})

describe('shared Demo teleoperation', () => {
  it('uses the same hold/release session and closes it when a task locks manual control', () => {
    vi.useFakeTimers()
    const view = render(<OperatorJoystick disabled={false} onError={vi.fn()} />)
    fireEvent.click(screen.getByText('解锁遥控'))
    const pad = screen.getByRole('group', { name: '底盘摇杆' })
    press(pad, 106, 0)
    const socket = TestSocket.instances[0]
    act(() => socket.open())
    expect(socket.sent.at(-1)?.linear).toBe(.1)
    fireEvent.pointerUp(pad, { pointerId: 1 })
    act(() => { vi.advanceTimersByTime(100) })
    expect(socket.sent.at(-1)).toEqual({ armed: true, linear: 0, angular: 0 })
    press(pad, 106, 0)
    expect(TestSocket.instances).toHaveLength(1)
    view.rerender(<OperatorJoystick disabled onError={vi.fn()} />)
    expect(socket.sent.at(-1)).toEqual({ armed: false, linear: 0, angular: 0 })
    expect(socket.readyState).toBe(3)
    expect(screen.getByText('任务中已锁定')).toBeTruthy()
    press(pad)
    expect(TestSocket.instances).toHaveLength(1)
  })
})
