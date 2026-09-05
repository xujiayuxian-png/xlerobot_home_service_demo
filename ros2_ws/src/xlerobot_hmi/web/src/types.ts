export type Readiness = 'READY' | 'DEGRADED' | 'BLOCKED'

export interface Task {
  task_id: string
  object_id: string
  source_place: string
  recipient_id: string
  grasp_backend: 'act' | 'centroid' | 'gpd'
  grasp_backend_used: string
  dry_run: boolean
  cancelable?: boolean
  status: string
  current_capability: string
  phase: string
  progress: number
  message: string
  error_code: number
  created_at: string
  updated_at: string
  completed_at: string
  stage_elapsed_s: number
  capability_durations: Record<string, number>
}

export interface Bootstrap {
  release: string
  unit: string
  default_dataset_id: string
  site: string
  workspace: string
  mapping_phase: string
  calibration_workflow: string
  calibration_capture_only?: boolean
  engineering_tools_enabled: boolean
  available_workspaces: string[]
  named_places: NamedPlace[]
  active_task: Task | null
  mapping: MappingState | null
  perception: PerceptionState | null
  voice_transcript: string
  collection: CollectionState | null
  drive_stop_latched: boolean | null
}

export interface PerceptionState {
  observation_id: string
  kind: 'object' | 'person'
  label: string
  camera_id: 'head'
  image_width: number
  image_height: number
  bbox: [number, number, number, number]
  confidence: number
  target: { frame_id: string, x: number, y: number, z: number }
}

export interface NamedPlace {
  id: string
  x: number
  y: number
  yaw: number
  nav_offset_m: number
}

export interface CollectionState {
  dataset_id: string
  episode_id: string
  template_id: 'pick' | 'manual'
  object_id: string
  dry_run: boolean
  status: string
  phase: string
  progress: number
  message: string
  elapsed_s: number
  frame_count: number
  episode_uri: string
  quality_passed: boolean
}

export interface MappingState {
  slam: string
  reset_notice?: string
  map: null | {
    frame_id: string
    width: number
    height: number
    resolution: number
    origin: { x: number, y: number, yaw: number }
    data: number[]
  }
  pose: null | { frame_id: string, x: number, y: number, yaw: number }
  scan: null | { frame_id: 'base_link', points_xy: [number, number][] }
  path: null | { frame_id: 'map', points_xy: [number, number][] }
}

export interface Health {
  readiness: Readiness
  execute_task_available: boolean
  voice_state: string
  drive_stop_latched: boolean | null
  diagnostics: Record<string, {
    level: number
    message: string
    hardware_id: string
    values: Record<string, string>
  }>
  requirements: Record<string, boolean>
}
