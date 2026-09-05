import { useEffect, useRef, useState, type PointerEvent } from 'react'

// Screen right means clockwise (negative ROS angular Z); screen up is forward.
export function joystickVector(x: number, y: number) {
  const distance = Math.hypot(x, y)
  if (distance <= 0.1) return { x: 0, y: 0, linear: 0, angular: 0 }
  const scale = Math.min(1, (distance - 0.1) / 0.9) / distance
  return { x: x / Math.max(1, distance), y: y / Math.max(1, distance),
    linear: -y * scale, angular: -x * scale }
}

export function MappingJoystick({ disabled, onMove, onRelease }: {
  disabled: boolean, onMove: (linear: number, angular: number) => void,
  onRelease: () => void,
}) {
  const [position, setPosition] = useState({ x: 0, y: 0 })
  const pointer = useRef<number | null>(null)
  const surface = useRef<HTMLDivElement>(null)
  const releaseCallback = useRef(onRelease)
  releaseCallback.current = onRelease

  const release = () => {
    const id = pointer.current
    if (id === null) return
    pointer.current = null
    setPosition({ x: 0, y: 0 })
    releaseCallback.current()
    if (surface.current?.hasPointerCapture(id)) surface.current.releasePointerCapture(id)
  }
  useEffect(() => { if (disabled) release() }, [disabled])
  useEffect(() => {
    const hidden = () => { if (document.hidden) release() }
    window.addEventListener('blur', release)
    document.addEventListener('visibilitychange', hidden)
    return () => {
      release()
      window.removeEventListener('blur', release)
      document.removeEventListener('visibilitychange', hidden)
    }
  }, [])

  const update = (event: PointerEvent<HTMLDivElement>) => {
    const bounds = event.currentTarget.getBoundingClientRect()
    const radius = Math.min(bounds.width, bounds.height) / 2 - 28
    if (radius <= 0) return
    const next = joystickVector((event.clientX - bounds.left - bounds.width / 2) / radius,
      (event.clientY - bounds.top - bounds.height / 2) / radius)
    setPosition(next)
    onMove(next.linear, next.angular)
  }
  return <div className="mapping-stick-wrap">
    <div ref={surface} className={`mapping-stick ${disabled ? 'disabled' : ''}`}
      role="group" aria-label="底盘摇杆" aria-disabled={disabled}
      onContextMenu={event => event.preventDefault()}
      onDragStart={event => event.preventDefault()}
      onPointerDown={event => {
        event.preventDefault()
        if (disabled || pointer.current !== null || event.button !== 0) return
        pointer.current = event.pointerId
        event.currentTarget.setPointerCapture(event.pointerId)
        update(event)
      }}
      onPointerMove={event => {
        if (pointer.current !== event.pointerId) return
        event.preventDefault()
        if (event.buttons === 0) { release(); return }
        update(event)
      }}
      onPointerUp={event => { if (pointer.current === event.pointerId) release() }}
      onPointerCancel={event => { if (pointer.current === event.pointerId) release() }}
      onLostPointerCapture={event => { if (pointer.current === event.pointerId) release() }}>
      <span className="joystick-forward">前进</span><span className="joystick-back">后退</span>
      <span className="joystick-left">左转</span><span className="joystick-right">右转</span>
      <span className="joystick-knob" style={{
        transform: `translate(${position.x * 78}px, ${position.y * 78}px)`,
      }} />
    </div>
    <p className="hint">按住拖动，松手停车；上/下前后行驶，左/右转向，可同时转弯。</p>
  </div>
}
