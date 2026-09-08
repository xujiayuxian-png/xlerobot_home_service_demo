import { cleanup, fireEvent, render, screen } from '@testing-library/react'
import { afterEach, describe, expect, it } from 'vitest'
import { App, preferNewestTask } from './App'
import { api } from './api'
import type { Task } from './types'

afterEach(cleanup)

describe('operator console API', () => {
  it('does not let a stale HTTP task response overwrite a terminal SSE state', () => {
    const base: Task = {
      task_id: 'task-1', object_id: 'ball', source_place: 'table',
      recipient_id: 'nearest_person', grasp_backend: 'act',
      grasp_backend_used: 'act', dry_run: false, status: 'RUNNING',
      current_capability: 'handover_object', phase: 'running', progress: .9,
      message: '', error_code: 0, created_at: '2026-07-20T00:00:00Z',
      updated_at: '2026-07-20T00:00:01Z', completed_at: '',
      stage_elapsed_s: 1, capability_durations: {},
    }
    const terminal = {
      ...base, status: 'SUCCEEDED',
      updated_at: '2026-07-20T00:00:02Z',
    }
    expect(preferNewestTask(terminal, base)?.status).toBe('SUCCEEDED')
  })

  it('uses the versioned fetch-deliver endpoint', async () => {
    let requested = ''
    globalThis.fetch = (async (input: RequestInfo | URL) => {
      requested = String(input)
      return new Response(JSON.stringify({ task_id: 'task-1' }), {
        status: 202,
        headers: { 'content-type': 'application/json' },
      })
    }) as typeof fetch
    await api.submitTask('羽毛球', 'gpd')
    expect(requested).toBe('/api/v1/tasks/fetch-deliver')
  })

  it('keeps normal collection End separate from Abort', async () => {
    const calls: Array<{ path: string, method: string }> = []
    globalThis.fetch = (async (input: RequestInfo | URL, init?: RequestInit) => {
      calls.push({ path: String(input), method: String(init?.method) })
      return new Response(JSON.stringify({
        dataset_id: 'dataset-1', episode_id: 'episode-1',
        template_id: 'manual', object_id: '', status: 'RUNNING',
        dry_run: false,
        phase: 'RECORDING', progress: 0.5, message: '',
        elapsed_s: 1, frame_count: 30, episode_uri: '',
        quality_passed: false,
      }), {
        status: 200,
        headers: { 'content-type': 'application/json' },
      })
    }) as typeof fetch

    await api.finalizeCollection('dataset-1', 'episode-1')
    await api.cancelCollection('dataset-1', 'episode-1')
    expect(calls).toEqual([
      {
        path: '/api/v1/datasets/dataset-1/episodes/episode-1/finalize',
        method: 'POST',
      },
      {
        path: '/api/v1/datasets/dataset-1/episodes/episode-1',
        method: 'DELETE',
      },
    ])
  })

  it('keeps normal End locked until collection reaches RECORDING', async () => {
    const collection = {
      dataset_id: 'dataset-1', episode_id: 'episode-1',
      template_id: 'manual', object_id: '', status: 'RUNNING',
      dry_run: false,
      phase: 'WAITING_HOME', progress: 0.38, message: '等待 Home',
      elapsed_s: 0, frame_count: 0, episode_uri: '',
      quality_passed: false,
    }
    const responses: Record<string, unknown> = {
      '/api/v1/bootstrap': {
        release: 'test', unit: 'robot-1', site: '', workspace: 'collection',
        default_dataset_id: 'xlerobot-glue-stick-grasp-30',
        mapping_phase: '', calibration_workflow: '',
        engineering_tools_enabled: true, available_workspaces: ['collection'],
        named_places: [], active_task: null, mapping: null, perception: null,
        voice_transcript: '', collection, drive_stop_latched: false,
      },
      '/api/v1/health': {
        readiness: 'READY', execute_task_available: false,
        voice_state: 'DISABLED', drive_stop_latched: false,
        diagnostics: {}, requirements: {
          collect_episode: true, collection_finalize: true,
          episode_review: true,
        },
      },
      '/api/v1/tasks': { tasks: [] },
    }
    globalThis.fetch = (async (input: RequestInfo | URL) =>
      new Response(JSON.stringify(responses[String(input)]), {
        status: 200, headers: { 'content-type': 'application/json' },
      })) as typeof fetch
    class SilentEventSource {
      onmessage: ((event: MessageEvent) => void) | null = null
      onerror: (() => void) | null = null
      onopen: (() => void) | null = null
      close() {}
    }
    globalThis.EventSource = SilentEventSource as unknown as typeof EventSource

    render(<App />)
    expect((await screen.findByPlaceholderText('Dataset ID') as HTMLInputElement).value)
      .toBe('xlerobot-glue-stick-grasp-30')
    const finish = await screen.findByRole(
      'button', { name: 'End / 结束并保存到本机' },
    ) as HTMLButtonElement
    const abort = screen.getByRole(
      'button', { name: 'Abort / 中止并保留 incomplete' },
    ) as HTMLButtonElement
    expect(finish.disabled).toBe(true)
    expect(abort.disabled).toBe(false)
  })

  it('never offers review controls for a dry-run collection', async () => {
    const collection = {
      dataset_id: 'dataset-1', episode_id: 'episode-1',
      template_id: 'manual', object_id: '', status: 'SUCCEEDED',
      dry_run: true, phase: 'COMPLETE', progress: 1, message: 'dry-run complete',
      elapsed_s: 1, frame_count: 0, episode_uri: 'dry-run://episode-1',
      quality_passed: true,
    }
    const responses: Record<string, unknown> = {
      '/api/v1/bootstrap': {
        release: 'test', unit: 'robot-1', site: '', workspace: 'collection',
        mapping_phase: '', calibration_workflow: '',
        engineering_tools_enabled: true, available_workspaces: ['collection'],
        named_places: [], active_task: null, mapping: null, perception: null,
        voice_transcript: '', collection, drive_stop_latched: false,
      },
      '/api/v1/health': {
        readiness: 'READY', execute_task_available: false,
        voice_state: 'DISABLED', drive_stop_latched: false,
        diagnostics: {}, requirements: {
          collect_episode: true, collection_finalize: true,
          episode_review: true,
        },
      },
      '/api/v1/tasks': { tasks: [] },
    }
    globalThis.fetch = (async (input: RequestInfo | URL) =>
      new Response(JSON.stringify(responses[String(input)]), {
        status: 200, headers: { 'content-type': 'application/json' },
      })) as typeof fetch
    class SilentEventSource {
      onmessage: ((event: MessageEvent) => void) | null = null
      onerror: (() => void) | null = null
      onopen: (() => void) | null = null
      close() {}
    }
    globalThis.EventSource = SilentEventSource as unknown as typeof EventSource

    render(<App />)
    await screen.findByText(/dry-run complete/)
    expect(screen.queryByRole('button', { name: '接受' })).toBeNull()
    expect(screen.queryByRole('button', { name: '拒绝' })).toBeNull()
  })

  it('shows restored hand-eye XY coverage and factual spans', async () => {
    const responses: Record<string, unknown> = {
      '/api/v1/bootstrap': {
        release: 'test', unit: 'robot-1', site: '', workspace: 'calibration',
        mapping_phase: '', calibration_workflow: 'right_handeye',
        engineering_tools_enabled: true, available_workspaces: ['calibration'],
        named_places: [], active_task: null, mapping: null, perception: null,
        voice_transcript: '', collection: null, drive_stop_latched: false,
      },
      '/api/v1/health': {
        readiness: 'READY', execute_task_available: false,
        voice_state: 'DISABLED', drive_stop_latched: false,
        diagnostics: {}, requirements: {},
      },
      '/api/v1/tasks': { tasks: [] },
      '/api/v1/calibrations/samples': {
        sample_count: 2,
        points: [
          { index: 0, x_m: -0.1, y_m: 0.2, z_m: 0.3 },
          { index: 1, x_m: 0.2, y_m: -0.2, z_m: 0.5 },
        ],
        spans_m: { x: 0.3, y: 0.4, z: 0.2 },
        max_pairwise_pose_angle_deg: 28.6,
      },
    }
    globalThis.fetch = (async (input: RequestInfo | URL) =>
      new Response(JSON.stringify(responses[String(input)]), {
        status: 200, headers: { 'content-type': 'application/json' },
      })) as typeof fetch
    class SilentEventSource {
      onmessage: ((event: MessageEvent) => void) | null = null
      onerror: (() => void) | null = null
      onopen: (() => void) | null = null
      close() {}
    }
    globalThis.EventSource = SilentEventSource as unknown as typeof EventSource

    const rendered = render(<App />)
    const plot = await screen.findByLabelText('右手眼 XY 姿态覆盖')
    const samples = plot.querySelectorAll('circle[data-sample-index]')
    expect(samples).toHaveLength(2)
    expect(samples[0].getAttribute('fill')).not.toBe(
      samples[1].getAttribute('fill'),
    )
    expect(screen.getByText('300.0 mm')).toBeTruthy()
    expect(screen.getByText('400.0 mm')).toBeTruthy()
    expect(screen.getByText('200.0 mm')).toBeTruthy()
    expect(screen.getByText('28.6°')).toBeTruthy()
    expect(rendered.getByText(/不据此增加或推断质量通过阈值/)).toBeTruthy()
  })

  it('keeps hand-eye capture controls isolated from bundle activation', async () => {
    const responses: Record<string, unknown> = {
      '/api/v1/bootstrap': {
        release: 'test', unit: 'robot-1', site: '', workspace: 'calibration',
        default_dataset_id: 'xlerobot-glue-stick-grasp-30',
        mapping_phase: '', calibration_workflow: 'right_handeye',
        calibration_capture_only: true,
        engineering_tools_enabled: true, available_workspaces: ['calibration'],
        named_places: [], active_task: null, mapping: null, perception: null,
        voice_transcript: '', collection: null, drive_stop_latched: false,
      },
      '/api/v1/health': {
        readiness: 'READY', execute_task_available: false,
        voice_state: 'DISABLED', drive_stop_latched: false,
        diagnostics: {}, requirements: {},
      },
      '/api/v1/tasks': { tasks: [] },
      '/api/v1/calibrations/samples': {
        sample_count: 0, points: [], spans_m: { x: 0, y: 0, z: 0 },
        max_pairwise_pose_angle_deg: 0,
      },
    }
    globalThis.fetch = (async (input: RequestInfo | URL) =>
      new Response(JSON.stringify(responses[String(input)]), {
        status: 200, headers: { 'content-type': 'application/json' },
      })) as typeof fetch
    class SilentEventSource {
      onmessage: ((event: MessageEvent) => void) | null = null
      onerror: (() => void) | null = null
      onopen: (() => void) | null = null
      close() {}
    }
    globalThis.EventSource = SilentEventSource as unknown as typeof EventSource

    render(<App />)
    expect(await screen.findByText('右臂手眼自动标定与验证')).toBeTruthy()
    expect(screen.getByRole('button', { name: '开始自动标定' })).toBeTruthy()
    expect(screen.queryByRole('button', { name: '采集当前静止姿态' })).toBeNull()
    expect(screen.queryByRole('button', { name: '检查当前工作流' })).toBeNull()
    expect(screen.queryByRole('button', { name: '求解并写入 draft' })).toBeNull()
    expect(screen.queryByRole('button', { name: '激活完整 calibration bundle' })).toBeNull()
  })

  it('keeps teleop disconnected in the default Demo view', async () => {
    const responses: Record<string, unknown> = {
      '/api/v1/bootstrap': {
        release: 'test', unit: 'robot-1', site: 'home', workspace: 'operator',
        mapping_phase: '', calibration_workflow: '',
        engineering_tools_enabled: false, available_workspaces: ['operator'],
        named_places: [], active_task: null, mapping: null, perception: null,
        voice_transcript: '', collection: null, drive_stop_latched: false,
      },
      '/api/v1/health': {
        readiness: 'READY', execute_task_available: true,
        voice_state: 'DISABLED', drive_stop_latched: false,
        diagnostics: {}, requirements: {
          execute_task_live_ready: true,
          head_camera: true, wrist_camera: true,
        },
      },
      '/api/v1/tasks': { tasks: [] },
    }
    globalThis.fetch = (async (input: RequestInfo | URL) =>
      new Response(JSON.stringify(responses[String(input)]), {
        status: 200, headers: { 'content-type': 'application/json' },
      })) as typeof fetch
    class SilentEventSource {
      onmessage: ((event: MessageEvent) => void) | null = null
      onerror: (() => void) | null = null
      onopen: (() => void) | null = null
      close() {}
    }
    globalThis.EventSource = SilentEventSource as unknown as typeof EventSource
    let socketCount = 0
    globalThis.WebSocket = class {
      static OPEN = 1
      static CLOSING = 2
      constructor() { socketCount += 1 }
    } as unknown as typeof WebSocket

    render(<App />)
    expect(await screen.findByText('发起取物递送')).toBeTruthy()
    expect(screen.queryByLabelText('底盘摇杆')).toBeNull()
    expect(socketCount).toBe(0)
    fireEvent.click(screen.getByRole('button', { name: '手动控制' }))
    expect(await screen.findByLabelText('底盘摇杆')).toBeTruthy()
    fireEvent.click(await screen.findByRole('button', { name: '解锁遥控' }))
    expect(socketCount).toBe(0)
  })
})
