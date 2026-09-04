import type { PerceptionState, Task } from './types'

const terminal = new Set(['SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED'])

export function activeCameraPerception(
  task: Task | null, perception: PerceptionState | null,
): PerceptionState | null {
  if (!task || terminal.has(task.status) || !perception) return null
  const capabilities = perception.kind === 'object'
    ? new Set(['detect_object', 'grasp_object'])
    : new Set(['scan_for_person'])
  return capabilities.has(task.current_capability) ? perception : null
}

export function activePersonTarget(
  task: Task | null, perception: PerceptionState | null,
): PerceptionState['target'] | null {
  if (!task || terminal.has(task.status) || perception?.kind !== 'person') return null
  return ['scan_for_person', 'approach_target'].includes(task.current_capability)
    ? perception.target : null
}

export function taskShowsNavigationPath(task: Task | null): boolean {
  return !!task && !terminal.has(task.status)
    && ['navigate_to_named_place', 'approach_target'].includes(task.current_capability)
}
