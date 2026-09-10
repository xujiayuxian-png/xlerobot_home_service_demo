import { act, cleanup, fireEvent, render, screen } from '@testing-library/react'
import { afterEach, beforeEach, expect, it, vi } from 'vitest'
import { App, CollectionWorkspace, MappingWorkspace } from './App'
import { CalibrationWorkbench } from './CalibrationWorkbench'
import { HeadCalibrationWorkspace } from './HeadCalibrationWorkspace'
import { ServoCalibrationWorkspace } from './ServoCalibrationWorkspace'
import { HoverReport } from './HoverReport'
import { api, type HeadCalibrationState, type ServoCalibrationState } from './api'
import { LanguageProvider, LanguageToggle } from './i18n'
import type { CollectionState } from './types'

const flush = async () => { await act(async () => {}) }
const en = () => fireEvent.click(screen.getByRole('button', { name: 'Switch to English' }))
const zh = () => fireEvent.click(screen.getByRole('button', { name: '切换为中文' }))
const noChineseUI = () => {
  const copy = document.body.cloneNode(true) as HTMLElement
  copy.querySelectorAll('.language-toggle').forEach(el => el.remove())
  expect(copy.textContent).not.toMatch(/\p{Script=Han}/u)
  for (const el of copy.querySelectorAll('*')) {
    for (const attr of ['aria-label', 'alt', 'placeholder', 'title']) {
      expect(el.getAttribute(attr) || '').not.toMatch(/\p{Script=Han}/u)
    }
  }
}
beforeEach(() => {
  vi.useFakeTimers()
  localStorage.clear()
  vi.spyOn(window, 'confirm').mockReturnValue(false)
})
afterEach(() => { cleanup(); vi.useRealTimers(); vi.unstubAllGlobals(); vi.restoreAllMocks() })

it('keeps collection fields, camera nodes and submitted dataset content unchanged on language switch', async () => {
  vi.spyOn(api, 'collectionEpisodes').mockResolvedValue({ episodes: [] })
  const start = vi.spyOn(api, 'startCollection').mockResolvedValue({ status: 'RUNNING' } as CollectionState)
  render(<LanguageProvider><LanguageToggle /><CollectionWorkspace state={null}
    initialDatasetId="trial" storage={{ root: '/example/地图', config_file: '/example/机器人.yaml' }}
    onState={vi.fn()} onError={vi.fn()} /></LanguageProvider>)
  await flush()
  const dataset = screen.getByLabelText('数据集名称')
  const instruction = screen.getByLabelText('语言指令') as HTMLInputElement
  const camera = screen.getByAltText('头部 D455 实时预览')
  en()
  expect(screen.getByLabelText('Dataset name')).toBe(dataset)
  expect(screen.getByLabelText('Language instruction')).toBe(instruction)
  expect(instruction.value).toBe('抓住羽毛球')
  expect(screen.getByAltText('Head D455 live preview')).toBe(camera)
  expect(screen.getByText('/example/地图/datasets/trial')).toBeTruthy()
  fireEvent.change(screen.getByPlaceholderText('Object label'), { target: { value: '黄色胶棒' } })
  fireEvent.change(instruction, { target: { value: '拿起黄色胶棒，保持竖直' } })
  zh(); en()
  expect(instruction.value).toBe('拿起黄色胶棒，保持竖直')
  expect(start).not.toHaveBeenCalled()
  fireEvent.click(screen.getByRole('button', { name: 'State-machine dry run' }))
  await flush()
  expect(start).toHaveBeenCalledExactlyOnceWith(expect.objectContaining({
    dataset_id: 'trial', object_id: '黄色胶棒', language_instruction: '拿起黄色胶棒，保持竖直', dry_run: true,
  }))
})

it('translates mapping controls and confirmation without changing the API operation or losing notices', async () => {
  vi.spyOn(api, 'site').mockResolvedValue({ active_version: '', draft: { places: [], map_saved: false } } as never)
  const reset = vi.spyOn(api, 'resetMap').mockResolvedValue({} as never)
  render(<LanguageProvider><LanguageToggle /><MappingWorkspace state={null} phase="build"
    initialSiteId="home" onError={vi.fn()} /></LanguageProvider>)
  await flush()
  const input = screen.getByLabelText('地图名')
  fireEvent.change(input, { target: { value: 'my-floor' } })
  en()
  expect(screen.getByLabelText('Map name')).toBe(input)
  noChineseUI()
  fireEvent.click(screen.getByRole('button', { name: 'Clear current map and rebuild' }))
  expect(window.confirm).toHaveBeenLastCalledWith(expect.stringContaining('Unsaved mapping results cannot be recovered'))
  expect(reset).not.toHaveBeenCalled()
  vi.mocked(window.confirm).mockReturnValue(true)
  fireEvent.click(screen.getByRole('button', { name: 'Clear current map and rebuild' }))
  await flush()
  expect(reset).toHaveBeenCalledTimes(1)
  expect(screen.getByRole('status').textContent).toContain('Current map cleared')
  zh()
  expect(screen.getByRole('status').textContent).toContain('当前地图已清除')
  expect((input as HTMLInputElement).value).toBe('my-floor')
})

const stages = Object.fromEntries(['servo', 'leader', 'head_camera', 'right_handeye', 'hover'].map(name =>
  [name, { present: true, quality_passed: true, freshness: 'current', missing: [] }]))
const snapshot = () => ({ unit: 'robot-1', active: 'v1', runtime_matches_active: true,
  hardware_enabled: true, draft_complete: true, state_path: '/example/units', stages,
  versions: ['v1'], session: { stage: null, running: false, generation: '', error: '', log_path: null } })

it('covers every workbench tab, translates native confirms and links to English docs without writes', async () => {
  const requests = vi.fn(async (path: string, _init?: RequestInit) => ({ ok: true,
    json: async () => path.endsWith('hover-result') ? { report: null, stale: false } : snapshot(),
  }))
  vi.stubGlobal('fetch', requests)
  render(<LanguageProvider><CalibrationWorkbench /></LanguageProvider>)
  await flush(); en()
  for (const label of ['Follower and head', 'Leader arm', 'Head camera', 'Hand-eye calibration', 'Hover accuracy', 'Results', 'Other calibration notes']) {
    fireEvent.click(screen.getByRole('button', { name: label }))
    await flush(); noChineseUI()
  }
  expect(screen.getByRole('link', { name: 'Config replacement and restoration' }).getAttribute('href')).toContain('?lang=en')
  fireEvent.click(screen.getByRole('button', { name: 'Hover accuracy' }))
  fireEvent.click(screen.getByRole('button', { name: 'Start and automatically validate three points' }))
  expect(window.confirm).toHaveBeenLastCalledWith(expect.stringContaining('Are the board and Tag 23 fixed'))
  expect(requests.mock.calls.every(([, init]) => !init?.method)).toBe(true)
  zh()
  expect(screen.getByRole('heading', { name: '机器人标定' })).toBeTruthy()
})

function visualState(): HeadCalibrationState {
  return { available: true, state_fresh: true, state_age_s: .1, action_ready: true,
    request_inflight: false, unit_id: 'robot-1', running: false, phase: 'IDLE',
    message: '', pose_index: -1, pose_count: 26, sample_count: 0, target_sample_count: 26,
    pose_states: Array(26).fill('pending'), pose_pan: Array(26).fill(0), pose_tilt: Array(26).fill(.8),
    pose_roles: Array.from({ length: 26 }, (_, i) => i < 20 ? 'fit' : 'validation'),
    result_uri: '', quality_passed: false, metrics: {},
    target: { accepted: true, fresh: true, age_s: .1, tag_count: 16, reprojection_rmse_px: .3 },
  }
}
it.each([false, true])('keeps live visual controls mounted and translates results after polling (handeye=%s)', async handeye => {
  let state = visualState()
  vi.spyOn(api, 'headCalibrationStatus').mockImplementation(async () => structuredClone(state))
  vi.spyOn(api, 'handeyeCalibrationStatus').mockImplementation(async () => structuredClone(state))
  const reset = vi.spyOn(api, 'resetVisualCalibration')
  const start = vi.spyOn(api, 'startHeadCalibration')
  render(<LanguageProvider><LanguageToggle /><HeadCalibrationWorkspace unitId="robot-1" handeye={handeye} onError={vi.fn()} /></LanguageProvider>)
  await flush()
  const camera = screen.getByRole('img'), checkbox = screen.getByRole('checkbox')
  fireEvent.click(checkbox)
  en(); noChineseUI()
  expect(screen.getByRole('checkbox')).toBe(checkbox)
  expect((checkbox as HTMLInputElement).checked).toBe(true)
  expect(screen.getByRole('img')).toBe(camera)
  state = { ...state, phase: 'COMPLETED', sample_count: 26, quality_passed: true,
    pose_states: Array(26).fill('captured'), result_uri: '/example/result.yaml',
    metrics: { translation_rmse_mm: 2, 'fit.translation_rmse_mm': 2, 'validation.translation_rmse_mm': 3 } }
  await act(async () => { await vi.advanceTimersByTimeAsync(600) })
  noChineseUI()
  fireEvent.click(screen.getByRole('button', { name: 'Archive and recalibrate' }))
  expect(window.confirm).toHaveBeenLastCalledWith(expect.stringContaining('Archive this run'))
  expect(reset).not.toHaveBeenCalled(); expect(start).not.toHaveBeenCalled()
  zh()
  expect(screen.getByRole('img').getAttribute('alt')).toBe('D455 标定板实时画面与 AprilTag 检测框')
})

it.each([false, true])('translates servo groups, joint hints and reset dialog (leader=%s)', async leader => {
  const group = leader ? 'leader' : 'right_arm'
  const joints = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper'].map((name, i) => ({
    name: `${group}.${name}`, servo_id: i + 1, position: 2048, zero: 2048, raw_min: 1024, raw_max: 3072,
    coverage: .8, zero_captured: true, reference_zero: 2048, zero_source: 'measured',
    range_captured: false, online: true, message: 'range sufficient',
  }))
  const state = { phase: 'PAUSED', active_group: group, joints, completed_groups: [], released_groups: [group], result_uri: '', session_uri: '' } as unknown as ServoCalibrationState
  vi.spyOn(api, 'servoCalibrationStatus').mockResolvedValue(state)
  const command = vi.spyOn(api, 'servoCalibrationStep')
  render(<LanguageProvider><LanguageToggle /><ServoCalibrationWorkspace unitId="robot-1" captureOnly onError={vi.fn()} onRestart={vi.fn()} /></LanguageProvider>)
  await flush(); en(); noChineseUI()
  fireEvent.click(screen.getByRole('button', { name: 'Recapture this group…' }))
  noChineseUI()
  expect(screen.getByRole('alertdialog').getAttribute('aria-label')).toBe(`Reset current capture for ${leader ? 'Leader arm' : 'Right arm'}`)
  expect(command).not.toHaveBeenCalled()
})

it('translates hover charts and advice while keeping download values and raw report IDs unchanged', async () => {
  const report = { id: 'run-1', status: 'COMPLETED', message: '三点验证完成，已回 ready；结果与建议补偿已保存，尚未应用补偿',
    updated_at: '2026-09-10', source: 'automatic', independent_arrivals: 3, frames: 60,
    arrivals: ['center', 'right', 'left'].map(target => ({ target, report_id: target, target_xyz_mm: [0, 0, -200], observed_xyz_mm: [3, -4, -160], planar_error_mm: 5, height_mm: 160, height_shortfall_mm: 40, max_joint_error_deg: 2 })),
    summary: { mean_planar_error_mm: 5, mean_height_shortfall_mm: 40 },
    suggestion: { frame: 'calibration_board', add_to_target_xyz_m: [-.003, .004, -.04], raise_target_mm: 40, between_pose_error_std_mm: [1, 2, 3], advice: '' },
  }
  vi.stubGlobal('fetch', vi.fn(async () => ({ ok: true, json: async () => ({ stale: false, report }) })))
  render(<LanguageProvider><LanguageToggle /><HoverReport /></LanguageProvider>)
  await flush(); en(); noChineseUI()
  expect(screen.getByRole('button', { name: 'Download advisory parameters (not applied)' })).toBeTruthy()
  const values = document.querySelector('pre')!.textContent
  zh(); expect(document.querySelector('pre')!.textContent).toBe(values)
})

it('switches the actual Demo and manual page without reconnecting SSE or altering task payload', async () => {
  const EventSource = vi.fn(function () { return { close: vi.fn(), onmessage: null } })
  vi.stubGlobal('EventSource', EventSource)
  const health = { readiness: 'READY', voice_state: 'WAITING_FOR_WAKE', execute_task_available: true,
    diagnostics: {}, requirements: { execute_task_live_ready: true, head_camera: true, wrist_camera: true } }
  vi.stubGlobal('fetch', vi.fn(async (path: string) => ({ ok: true, json: async () => path.endsWith('bootstrap') ? {
    release: 'v1', unit: 'robot-1', site: 'home', available_workspaces: ['operator'],
    active_task: null, mapping: null, perception: null, collection: null, drive_stop_latched: false,
    named_places: [{ id: 'table', x: 0, y: 0, yaw: 0 }],
  } : path.endsWith('health') ? health : { tasks: [] } })))
  const submit = vi.spyOn(api, 'submitTask').mockResolvedValue(null as never)
  render(<LanguageProvider><App /></LanguageProvider>)
  await flush()
  const input = screen.getByRole('textbox', { name: '物体名' })
  fireEvent.change(input, { target: { value: '黄色胶棒' } })
  fireEvent.change(screen.getByRole('combobox', { name: '抓取路线' }), { target: { value: 'gpd' } })
  en(); noChineseUI()
  expect(screen.getByRole('textbox', { name: 'Object' })).toBe(input)
  expect(submit).not.toHaveBeenCalled()
  fireEvent.click(screen.getByRole('button', { name: 'Start task' }))
  await flush()
  expect(submit).toHaveBeenCalledExactlyOnceWith('黄色胶棒', 'gpd')
  fireEvent.click(screen.getByRole('button', { name: 'Manual control' }))
  noChineseUI()
  zh(); en()
  expect(EventSource).toHaveBeenCalledTimes(1)
})
