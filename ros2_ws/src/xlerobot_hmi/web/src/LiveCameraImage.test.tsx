import { act, cleanup, fireEvent, render, screen } from '@testing-library/react'
import { afterEach, beforeEach, expect, it, vi } from 'vitest'
import { LiveCameraImage } from './LiveCameraImage'

const props = { src: '/api/v1/cameras/detection/stream', alt: 'camera' }
const advance = async (ms: number) => { await act(async () => { await vi.advanceTimersByTimeAsync(ms) }) }
beforeEach(() => vi.useFakeTimers())
afterEach(() => { cleanup(); vi.useRealTimers() })

it('waits for service readiness and starts without refreshing the page', () => {
  const { rerender } = render(<LiveCameraImage {...props} ready={false} />)
  const image = screen.getByRole('img')
  expect(image.getAttribute('src')).toBeNull()
  rerender(<LiveCameraImage {...props} ready />)
  expect(image.getAttribute('src')).toBe(props.src)
})

it('retries an initial 503/connection error without remounting the image', async () => {
  render(<LiveCameraImage {...props} />)
  const image = screen.getByRole('img')
  fireEvent.error(image)
  fireEvent.error(image)
  await advance(1200)
  expect(screen.getByRole('img')).toBe(image)
  expect(image.getAttribute('src')).toBe(`${props.src}?camera_retry=1`)
  fireEvent.error(image)
  await advance(1200)
  expect(image.getAttribute('src')).toBe(`${props.src}?camera_retry=2`)
})

it('does not periodically reload a healthy stream on status updates', async () => {
  const { rerender } = render(<LiveCameraImage {...props} ready />)
  const image = screen.getByRole('img')
  fireEvent.load(image)
  for (let i = 0; i < 10; i++) {
    rerender(<LiveCameraImage {...props} ready />)
    await advance(1000)
  }
  expect(image.getAttribute('src')).toBe(props.src)
})

it('cancels retry when ready goes away, and reconnects on recovery', async () => {
  const { rerender, unmount } = render(<LiveCameraImage {...props} ready />)
  const image = screen.getByRole('img')
  fireEvent.error(image)
  rerender(<LiveCameraImage {...props} ready={false} />)
  await advance(2400)
  expect(image.getAttribute('src')).toBeNull()
  rerender(<LiveCameraImage {...props} ready />)
  expect(image.getAttribute('src')).toBe(props.src)
  fireEvent.error(image)
  unmount()
  expect(vi.getTimerCount()).toBe(0)
})
