import { cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react'
import { afterEach, describe, expect, it, vi } from 'vitest'
import { CollectionWorkspace } from './App'
import { api } from './api'
import type { CollectionState } from './types'

afterEach(() => { cleanup(); vi.restoreAllMocks() })
const waiting = {
  dataset_id: 'trial', episode_id: 'episode-1', template_id: 'pick',
  object_id: '羽毛球', dry_run: false, status: 'RUNNING', phase: 'WAITING_HOME',
  progress: .38, message: '', elapsed_s: 0, frame_count: 0,
  episode_uri: '', quality_passed: false,
} as CollectionState

describe('two-stage collection controls', () => {
  it('requires Reset after release and never starts preparation from Reset', async () => {
    const recover = vi.spyOn(api, 'recoverCollection').mockResolvedValue({
      message: 'torque off', collection: null,
    })
    const start = vi.spyOn(api, 'startCollection')
    render(<CollectionWorkspace state={null} initialDatasetId="trial" onState={vi.fn()} onError={vi.fn()} />)
    fireEvent.click(screen.getByRole('button', { name: '释放主从臂扭矩' }))
    await waitFor(() => expect(screen.getByRole('status').textContent).toBe('torque off'))
    expect((screen.getByRole('button', { name: '开始 / 准备 pregrasp' }) as HTMLButtonElement).disabled).toBe(true)
    fireEvent.click(screen.getByRole('button', { name: 'Reset / 重置状态' }))
    await waitFor(() => expect(recover).toHaveBeenLastCalledWith('reset'))
    await waitFor(() => expect((screen.getByRole('button', { name: '开始 / 准备 pregrasp' }) as HTMLButtonElement).disabled).toBe(false))
    expect(start).not.toHaveBeenCalled()
  })

  it('updates instruction when object changes and still allows editing it', () => {
    render(<CollectionWorkspace state={null} initialDatasetId="trial" onState={vi.fn()} onError={vi.fn()} />)
    fireEvent.change(screen.getByPlaceholderText('物体标签'), { target: { value: '黄色胶棒' } })
    expect((screen.getByLabelText('语言指令') as HTMLInputElement).value).toBe('抓住黄色胶棒')
    fireEvent.change(screen.getByLabelText('语言指令'), { target: { value: '拿起黄色胶棒' } })
    expect((screen.getByLabelText('语言指令') as HTMLInputElement).value).toBe('拿起黄色胶棒')
  })

  it('Start only submits preparation, Home is separate and waits for readiness', async () => {
    const start = vi.spyOn(api, 'startCollection').mockResolvedValue(waiting)
    const home = vi.spyOn(api, 'beginCollection').mockResolvedValue(waiting)
    const props = { initialDatasetId: 'trial', onState: vi.fn(), onError: vi.fn() }
    const view = render(<CollectionWorkspace {...props} state={null} />)
    fireEvent.click(screen.getByRole('button', { name: '开始 / 准备 pregrasp' }))
    await waitFor(() => expect(start).toHaveBeenCalledTimes(1))
    expect(home).not.toHaveBeenCalled()
    view.rerender(<CollectionWorkspace {...props} state={{ ...waiting, phase: 'PREPARE_PREGRASP' }} />)
    expect((screen.getByRole('button', { name: 'Home / 释放主臂并开始采集' }) as HTMLButtonElement).disabled).toBe(true)
    view.rerender(<CollectionWorkspace {...props} state={waiting} />)
    expect((screen.getByRole('button', { name: 'End / 结束并保存到本机' }) as HTMLButtonElement).disabled).toBe(true)
    fireEvent.click(screen.getByRole('button', { name: 'Home / 释放主臂并开始采集' }))
    await waitFor(() => expect(home).toHaveBeenCalledWith('trial', 'episode-1'))
  })

  it('gates keyboard Home/End by phase and ignores typing and repeat', async () => {
    const home = vi.spyOn(api, 'beginCollection').mockResolvedValue(waiting)
    const end = vi.spyOn(api, 'finalizeCollection').mockResolvedValue(waiting)
    const props = { initialDatasetId: 'trial', onState: vi.fn(), onError: vi.fn() }
    const view = render(<CollectionWorkspace {...props} state={waiting} />)
    fireEvent.keyDown(screen.getByLabelText('语言指令'), { key: 'Home' })
    fireEvent.keyDown(window, { key: 'Home', repeat: true })
    expect(home).not.toHaveBeenCalled()
    fireEvent.keyDown(window, { key: 'Home' })
    await waitFor(() => expect(home).toHaveBeenCalledTimes(1))
    view.rerender(<CollectionWorkspace {...props} state={{ ...waiting, phase: 'STARTING_RECORDING' }} />)
    fireEvent.keyDown(window, { key: 'Home' })
    fireEvent.keyDown(window, { key: 'End' })
    expect(home).toHaveBeenCalledTimes(1)
    expect(end).not.toHaveBeenCalled()
    view.rerender(<CollectionWorkspace {...props} state={{ ...waiting, phase: 'RECORDING' }} />)
    fireEvent.keyDown(window, { key: 'End' })
    await waitFor(() => expect(end).toHaveBeenCalledTimes(1))
  })
})
