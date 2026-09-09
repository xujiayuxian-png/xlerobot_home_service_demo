import { useEffect, useRef, useState } from 'react'

/** MJPEG reconnects belong to the image, never to the calibration workspace. */
export function LiveCameraImage({ src, alt, className, ready = true }: {
  src: string, alt: string, className?: string, ready?: boolean,
}) {
  const [attempt, setAttempt] = useState(0)
  const timer = useRef<ReturnType<typeof setTimeout> | null>(null)
  const enabled = useRef(ready)
  enabled.current = ready

  const clearRetry = () => {
    if (timer.current !== null) clearTimeout(timer.current)
    timer.current = null
  }
  useEffect(() => {
    // A readiness transition removes/reinstates src, initiating a fresh request.
    // Ordinary status polls with the same readiness do not touch a healthy stream.
    clearRetry()
    return clearRetry
  }, [src, ready])

  const retry = () => {
    if (!enabled.current || timer.current !== null) return
    timer.current = setTimeout(() => {
      timer.current = null
      if (enabled.current) setAttempt(value => value + 1)
    }, 1200)
  }
  const url = attempt ? `${src}${src.includes('?') ? '&' : '?'}camera_retry=${attempt}` : src
  return <img className={className} src={ready ? url : undefined} alt={alt}
    onError={retry} onLoad={clearRetry} />
}
