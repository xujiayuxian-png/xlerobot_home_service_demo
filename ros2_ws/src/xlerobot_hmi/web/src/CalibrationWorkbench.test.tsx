import { act, cleanup, fireEvent, render, screen } from '@testing-library/react'
import { afterEach, beforeEach, expect, it, vi } from 'vitest'
import { CalibrationWorkbench } from './CalibrationWorkbench'

vi.mock('./ServoCalibrationWorkspace', () => ({ ServoCalibrationWorkspace: () => <div data-testid="servo-workspace">servo controls</div> }))
vi.mock('./HeadCalibrationWorkspace', () => ({ HeadCalibrationWorkspace: () => <div data-testid="visual-workspace">visual controls</div> }))
const stages = Object.fromEntries(['servo', 'leader', 'head_camera', 'right_handeye', 'hover'].map(name =>
  [name, { present: true, quality_passed: true, freshness: 'needs_validation', missing: [] }]))
let snapshot: Record<string, unknown>
const requests: Array<{ path: string, body: Record<string, unknown> }> = []
const flush = async () => { await act(async () => {}) }
beforeEach(() => {
  vi.useFakeTimers()
  requests.length = 0
  snapshot = { unit: 'test-unit', active: 'original', runtime_matches_active: true,
    draft_complete: false, hardware_enabled: true, state_path: '/state/units/test-unit',
    session: { stage: null, running: false, generation: '', error: '', log_path: null },
    stages, versions: ['original', 'review'] }
  vi.stubGlobal('fetch', vi.fn(async (path: string, init?: RequestInit) => {
    if (init?.method === 'POST') requests.push({ path, body: JSON.parse(String(init.body)) })
    return { ok: true, json: async () => structuredClone(snapshot) }
  }))
  vi.spyOn(window, 'confirm').mockReturnValue(true)
})
afterEach(() => { cleanup(); vi.useRealTimers(); vi.unstubAllGlobals(); vi.restoreAllMocks() })

it('browsing every tab is read-only and never starts or stops hardware', async () => {
  render(<CalibrationWorkbench />)
  await flush()
  for (const name of ['Leader 主臂', '头部相机', '手眼标定', '悬停精度', '结果管理', '其他标定说明']) {
    fireEvent.click(screen.getByRole('button', { name }))
  }
  await flush()
  expect(requests).toEqual([])
})

it('starts Leader only after explicit confirmation; default resumes, never discards samples', async () => {
  render(<CalibrationWorkbench />)
  await flush()
  fireEvent.click(screen.getByRole('button', { name: 'Leader 主臂' }))
  fireEvent.click(screen.getByRole('button', { name: '启动此项设备会话' }))
  await flush()
  expect(requests).toEqual([{ path: '/workbench-api/start', body: { stage: 'leader', fresh: false, confirmed: true } }])
})

it('starts the robot-side automatic hover job only on confirmed click', async () => {
  render(<CalibrationWorkbench />)
  await flush()
  fireEvent.click(screen.getByRole('button', { name: /^悬停精度$/ }))
  await flush()
  expect(requests).toEqual([])
  vi.mocked(window.confirm).mockReturnValue(false)
  fireEvent.click(screen.getByRole('button', { name: '一键启动并自动验证三点' }))
  expect(requests).toEqual([])
  vi.mocked(window.confirm).mockReturnValue(true)
  fireEvent.click(screen.getByRole('button', { name: '一键启动并自动验证三点' }))
  await flush()
  expect(requests).toEqual([{ path: '/workbench-api/auto-hover', body: { confirmed: true } }])
})

it('does not perform canceled start or stop', async () => {
  vi.mocked(window.confirm).mockReturnValue(false)
  render(<CalibrationWorkbench />)
  await flush()
  fireEvent.click(screen.getByRole('button', { name: '启动此项设备会话' }))
  expect(requests).toHaveLength(0)
})

it('keeps current controls mounted across polling and browsing; replacement disabled while occupied', async () => {
  snapshot.session = { stage: 'servo', running: true, generation: 'session-one', error: '', log_path: null }
  render(<CalibrationWorkbench />)
  await flush()
  const original = screen.getByTestId('servo-workspace')
  fireEvent.click(screen.getByRole('button', { name: '结果管理' }))
  await act(async () => { await vi.advanceTimersByTimeAsync(1600) })
  expect(screen.getByTestId('servo-workspace')).toBe(original)
  expect((screen.getByRole('button', { name: '预览替换' }) as HTMLButtonElement).disabled).toBe(true)
  expect((screen.getByRole('button', { name: '恢复所选版本' }) as HTMLButtonElement).disabled).toBe(true)
  expect(requests).toHaveLength(0)
})

it('read-only mode disables session start', async () => {
  snapshot.hardware_enabled = false
  render(<CalibrationWorkbench />)
  await flush()
  expect((screen.getByRole('button', { name: '启动此项设备会话' }) as HTMLButtonElement).disabled).toBe(true)
})

it('recovers from hover startup and connection failures without hiding operation errors', async () => {
  snapshot.session = { stage: 'hover', running: true, generation: 'hover-one', error: '', log_path: null }
  let available = false
  vi.stubGlobal('fetch', vi.fn(async (path: string, init?: RequestInit) => {
    if (path === '/api/v1/hover/status') return available
      ? { ok: true, json: async () => ({ phase: 'IDLE', message: '设备就绪', ready: true, busy: false, plan: null }) }
      : { ok: false, status: 503, text: async (): Promise<string> => '标定服务正在启动，请稍候' }
    if (init?.method === 'POST') return { ok: false, text: async (): Promise<string> => '目标规划失败' }
    return { ok: true, json: async () => path === '/workbench-api/hover-result' ? { report: null } : structuredClone(snapshot) }
  }))
  render(<CalibrationWorkbench />)
  await flush()
  fireEvent.click(screen.getByRole('button', { name: /^悬停精度$/ }))
  await flush()
  expect(screen.getByText(/正在启动悬停服务，连接成功/)).toBeTruthy()
  expect(screen.queryByRole('alert')).toBeNull()
  await act(async () => { await vi.advanceTimersByTimeAsync(31000) })
  expect(screen.getByRole('alert').textContent).toContain('暂时无法连接')
  available = true
  await act(async () => { await vi.advanceTimersByTimeAsync(800) })
  expect(screen.queryByRole('alert')).toBeNull()
  expect(screen.queryByText(/正在启动悬停服务，连接成功/)).toBeNull()
  fireEvent.click(screen.getByRole('button', { name: '自动运行三点验证' }))
  await flush()
  await act(async () => { await vi.advanceTimersByTimeAsync(800) })
  expect(screen.getByRole('alert').textContent).toContain('目标规划失败')
  available = false
  await act(async () => { await vi.advanceTimersByTimeAsync(800) })
  expect((screen.getByRole('button', { name: '自动运行三点验证' }) as HTMLButtonElement).disabled).toBe(true)
  available = true
  await act(async () => { await vi.advanceTimersByTimeAsync(800) })
  expect(screen.getAllByRole('alert')).toHaveLength(1)
  expect(screen.getByRole('alert').textContent).toContain('目标规划失败')
})
