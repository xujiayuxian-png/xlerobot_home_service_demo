import type { Bootstrap, CollectionState, Health, MappingState, NamedPlace, Task } from './types'

export interface SiteSummary {
  site_id: string
  active_version: string
  draft: {
    map_saved: boolean, map_name: string, map_saved_at: string,
    places: Array<NamedPlace & { dock: boolean, validated: boolean }>,
    ready: boolean,
  }
}

export type CalibrationCoverage = {
  sample_count: number
  points: Array<{ index: number, x_m: number, y_m: number, z_m: number }>
  spans_m: { x: number, y: number, z: number }
  max_pairwise_pose_angle_deg: number
}

async function request<T>(path: string, init?: RequestInit): Promise<T> {
  const response = await fetch(path, {
    ...init,
    headers: { 'content-type': 'application/json', ...init?.headers },
  })
  if (!response.ok) {
    throw new Error((await response.text()) || `HTTP ${response.status}`)
  }
  return response.json() as Promise<T>
}

export const api = {
  recoverCollection: (operation: 'release' | 'reset') =>
    request<{ message: string, collection: CollectionState | null }>(
      `/api/v1/collections/${operation}`, { method: 'POST', body: '{}' }),
  bootstrap: () => request<Bootstrap>('/api/v1/bootstrap'),
  health: () => request<Health>('/api/v1/health'),
  recentTasks: () => request<{ tasks: Task[] }>('/api/v1/tasks'),
  submitTask: (objectId: string, graspBackend: 'act' | 'centroid' | 'gpd') =>
    request<Task>('/api/v1/tasks/fetch-deliver', {
    method: 'POST',
    body: JSON.stringify({
      object_id: objectId,
      source_place: 'table',
      recipient_id: 'nearest_person',
      grasp_backend: graspBackend,
      dry_run: false,
    }),
    }),
  cancelTask: (taskId: string) => request<Task>(`/api/v1/tasks/${taskId}`, {
    method: 'DELETE',
  }),
  operatorLocalize: () => request<{
    message: string, position_stddev_m: number, yaw_stddev_rad: number
  }>('/api/v1/operator/localize', { method: 'POST', body: '{}' }),
  operatorNavigate: (placeId: string) => request<{ message: string, place_id: string }>(
    '/api/v1/operator/navigate', {
      method: 'POST', body: JSON.stringify({ place_id: placeId }),
    },
  ),
  operatorStopBase: () => request<{
    status: string, latched: boolean, kind: 'base_software_inhibit',
    is_emergency_stop: false, canceled_goal_count: number
  }>(
    '/api/v1/operator/stop-base', { method: 'POST', body: '{}' },
  ),
  operatorClearBaseStop: () => request<{
    status: string, latched: boolean, kind: 'base_software_inhibit',
    is_emergency_stop: false
  }>('/api/v1/operator/stop-base', { method: 'DELETE' }),
  operatorPreset: (preset: 'arm_ready' | 'head_ready' | 'gripper_open') =>
    request<{ preset: string, message: string }>('/api/v1/operator/preset', {
      method: 'POST', body: JSON.stringify({ preset }),
    }),
  mappingState: () => request<MappingState>('/api/v1/mapping/state'),
  resetMap: () => request<{ status: 'reset', saved_assets_preserved: boolean, mapping: MappingState }>(
    '/api/v1/mapping/reset', { method: 'POST', body: JSON.stringify({ confirm: true }) },
  ),
  site: (siteId: string) => request<SiteSummary>(
    `/api/v1/sites?site_id=${encodeURIComponent(siteId)}`,
  ),
  saveMap: (siteId: string, mapName: string) => request<{
    artifact_uri: string, version: string
  }>('/api/v1/mapping/sessions', {
    method: 'POST', body: JSON.stringify({ site_id: siteId, map_name: mapName }),
  }),
  setPlace: (siteId: string, placeId: string, dock: boolean, navOffset: number) =>
    request<{ artifact_uri: string }>(`/api/v1/sites/${siteId}/places`, {
      method: 'POST', body: JSON.stringify({
        place_id: placeId, dock, nav_offset_m: navOffset,
      }),
    }),
  removePlace: (siteId: string, placeId: string) =>
    request<{ artifact_uri: string }>(
      `/api/v1/sites/${siteId}/places/${encodeURIComponent(placeId)}`,
      { method: 'DELETE' },
    ),
  validateLocalization: (siteId: string) => request<{
    position_stddev_m: number, yaw_stddev_rad: number, message: string
  }>(`/api/v1/sites/${siteId}/validate-localization`, {
    method: 'POST', body: '{}',
  }),
  validatePlace: (siteId: string, placeId: string) => request<{
    place_id: string, position_error_m: number, yaw_error_rad: number,
    duration_s: number, site_ready: boolean, artifact_uri: string
  }>(`/api/v1/sites/${siteId}/validate-place`, {
    method: 'POST', body: JSON.stringify({ place_id: placeId }),
  }),
  activateSite: (siteId: string, version = '') =>
    request<{ artifact_uri: string }>(`/api/v1/sites/${siteId}/activate`, {
      method: 'POST', body: JSON.stringify({ version }),
    }),
  calibrationPreflight: (unitId: string, workflowId: string) =>
    request<{ artifact_uri: string, quality_passed: boolean, message: string }>(
      '/api/v1/calibrations/jobs', {
        method: 'POST', body: JSON.stringify({
          unit_id: unitId, workflow_id: workflowId,
          automatic: false, dry_run: true,
        }),
      },
    ),
  importCalibration: (unitId: string, workflowId: string, sourceUri: string) =>
    request<{ draft_uri: string, quality_passed: boolean, metrics: Record<string, number> }>(
      '/api/v1/calibrations/imports', {
        method: 'POST', body: JSON.stringify({
          unit_id: unitId, workflow_id: workflowId, source_uri: sourceUri,
        }),
      },
    ),
  activateCalibration: (unitId: string, version = '') =>
    request<{ artifact_uri: string }>(`/api/v1/calibrations/${unitId}/activate`, {
      method: 'POST', body: JSON.stringify({ version }),
    }),
  saveBaseGeometry: (payload: Record<string, unknown>) =>
    request<{ draft_uri: string, quality_passed: boolean, metrics: Record<string, number> }>(
      '/api/v1/calibrations/base-geometry', {
        method: 'POST', body: JSON.stringify(payload),
      },
    ),
  captureCalibrationSample: (unitId: string) =>
    request<{ sample_count: number, message: string }>('/api/v1/calibrations/samples', {
      method: 'POST', body: JSON.stringify({ unit_id: unitId }),
    }),
  calibrationSampleCoverage: () =>
    request<CalibrationCoverage>('/api/v1/calibrations/samples'),
  solveCalibrationSamples: (unitId: string) =>
    request<{
      draft_uri: string, quality_passed: boolean, sample_count: number,
      metrics: Record<string, number>
    }>('/api/v1/calibrations/solve', {
      method: 'POST', body: JSON.stringify({ unit_id: unitId }),
    }),
  moveCalibrationPose: (poseIndex: number) =>
    request<{
      pose_index: number, pose_name: string, pose_count: number, message: string
    }>('/api/v1/calibrations/move-pose', {
      method: 'POST', body: JSON.stringify({ pose_index: poseIndex }),
    }),
  servoCalibrationStep: (
    unitId: string, command: string, group: string, joint: string,
  ) => request<any>('/api/v1/calibrations/servo/step', {
    method: 'POST', body: JSON.stringify({ unit_id: unitId, command, group, joint }),
  }),
  startCollection: (payload: Record<string, unknown>) =>
    request<CollectionState>('/api/v1/collections/sessions', {
      method: 'POST', body: JSON.stringify(payload),
    }),
  finalizeCollection: (datasetId: string, episodeId: string) =>
    request<CollectionState>(
      `/api/v1/datasets/${encodeURIComponent(datasetId)}/episodes/`
      + `${encodeURIComponent(episodeId)}/finalize`, {
      method: 'POST', body: '{}',
    }),
  cancelCollection: (datasetId: string, episodeId: string) =>
    request<CollectionState>(
      `/api/v1/datasets/${encodeURIComponent(datasetId)}/episodes/`
      + encodeURIComponent(episodeId), {
      method: 'DELETE',
    }),
  beginCollection: (datasetId: string, episodeId: string) =>
    request<CollectionState>(
      `/api/v1/datasets/${encodeURIComponent(datasetId)}/episodes/`
      + `${encodeURIComponent(episodeId)}/begin`, { method: 'POST', body: '{}' }),
  reviewEpisode: (
    datasetId: string, episodeId: string,
    status: 'accepted' | 'rejected', notes = '',
  ) =>
    request<{ review_uri: string }>(
      `/api/v1/datasets/${encodeURIComponent(datasetId)}/episodes/`
      + `${encodeURIComponent(episodeId)}/review`, {
      method: 'POST', body: JSON.stringify({ status, notes, failure_reason: '' }),
    }),
}
