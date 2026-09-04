import { describe, expect, it } from 'vitest'
import { upsertTaskHistory } from './taskHistory'
import type { Task } from './types'

const task = (task_id: string, updated_at: string): Task => ({
  task_id, object_id: task_id, source_place: 'table', recipient_id: 'nearest_person',
  dry_run: false, status: 'RUNNING', current_capability: 'detect_object',
  phase: 'running', progress: .5, message: '', error_code: 0,
  created_at: updated_at, updated_at, completed_at: '', stage_elapsed_s: 1,
  capability_durations: {},
})

describe('task history updates', () => {
  it('replaces a live task immediately and keeps newest tasks first', () => {
    const old = task('voice-task', '2026-07-15T00:00:00+00:00')
    const latest = { ...old, status: 'FAILED', updated_at: '2026-07-15T00:01:00+00:00' }
    const result = upsertTaskHistory(
      [task('older-task', '2026-07-14T23:00:00+00:00'), old], latest,
    )
    expect(result.map(item => item.task_id)).toEqual(['voice-task', 'older-task'])
    expect(result[0].status).toBe('FAILED')
  })
})
