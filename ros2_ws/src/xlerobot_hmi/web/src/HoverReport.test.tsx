import { cleanup, render, screen } from '@testing-library/react'
import { afterEach, expect, it, vi } from 'vitest'
import { HoverReport } from './HoverReport'

afterEach(() => { cleanup(); vi.unstubAllGlobals() })
function mock(stale = false) {
  const fetch = vi.fn(async (_path: string) => ({ ok: true, json: async () => ({ stale, report: {
    id: 'run-1', status: 'COMPLETED', message: '已完成', source: 'automatic', updated_at: '2026-09-10',
    independent_arrivals: 3, frames: 60,
    arrivals: ['center', 'right', 'left'].map(target => ({ target, report_id: target,
      target_xyz_mm: [0, 0, -200], observed_xyz_mm: [3, -4, -160],
      planar_error_mm: 5, height_mm: 160, height_shortfall_mm: 40, max_joint_error_deg: 2 })),
    summary: { mean_planar_error_mm: 5, mean_height_shortfall_mm: 40 },
    suggestion: { frame: 'calibration_board', add_to_target_xyz_m: [-.003, .004, -.04],
      raise_target_mm: 40, between_pose_error_std_mm: [1, 2, 3], advice: '先重新验证' },
  } }) }))
  vi.stubGlobal('fetch', fetch)
  return fetch
}

it('shows saved charts and signed advisory even with no device session; never commands motion', async () => {
  const fetch = mock()
  render(<HoverReport />)
  await screen.findByText('测量完成 · 补偿尚未验证')
  expect(screen.getByText('3 / 3')).toBeTruthy()
  expect(screen.getByText('40.0 mm', { selector: 'strong' })).toBeTruthy()
  expect(screen.getByText(/add_to_target_xyz_m: \[-0.00300, 0.00400, -0.04000\]/)).toBeTruthy()
  expect(screen.getAllByRole('img')).toHaveLength(2)
  expect(screen.getByRole('button', { name: '下载建议参数（不应用）' })).toBeTruthy()
  expect(fetch.mock.calls.every(args => args.length === 1)).toBe(true)
})

it('keeps historical measurements but hides invalid compensation after calibration changes', async () => {
  mock(true)
  render(<HoverReport />)
  await screen.findByRole('alert')
  expect(screen.getByText(/历史建议已失效/)).toBeTruthy()
  expect(screen.queryByRole('button', { name: '下载建议参数（不应用）' })).toBeNull()
  expect(screen.getAllByRole('img')).toHaveLength(2)
})
