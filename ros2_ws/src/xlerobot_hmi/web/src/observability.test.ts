import { describe, expect, it } from 'vitest'
import {
  activeCameraPerception, activePersonTarget, taskShowsNavigationPath,
} from './observability'
import type { PerceptionState, Task } from './types'

const task = (capability: string, status = 'RUNNING') => ({
  task_id: '1', object_id: '羽毛球', source_place: 'table',
  recipient_id: 'nearest_person', dry_run: false, status,
  current_capability: capability, phase: '', progress: 0, message: '',
  error_code: 0, created_at: '', updated_at: '', completed_at: '',
  stage_elapsed_s: 0, capability_durations: {},
}) satisfies Task

const observation = (kind: 'object' | 'person') => ({
  observation_id: 'obs', kind, label: kind, camera_id: 'head',
  image_width: 640, image_height: 480, bbox: [1, 2, 3, 4], confidence: .9,
  target: { frame_id: 'map', x: 1, y: 2, z: 0 },
}) satisfies PerceptionState

describe('task-scoped observability', () => {
  it('shows observations only while their result is in use', () => {
    expect(activeCameraPerception(task('grasp_object'), observation('object'))).not.toBeNull()
    expect(activeCameraPerception(task('scan_for_person'), observation('object'))).toBeNull()
    expect(activeCameraPerception(task('scan_for_person'), observation('person'))).not.toBeNull()
    expect(activeCameraPerception(task('approach_target'), observation('person'))).toBeNull()
    expect(activePersonTarget(task('approach_target'), observation('person'))).not.toBeNull()
    expect(activePersonTarget(task('complete', 'SUCCEEDED'), observation('person'))).toBeNull()
  })

  it('shows a path only during task navigation stages', () => {
    expect(taskShowsNavigationPath(task('navigate_to_named_place'))).toBe(true)
    expect(taskShowsNavigationPath(task('approach_target'))).toBe(true)
    expect(taskShowsNavigationPath(task('grasp_object'))).toBe(false)
  })
})
