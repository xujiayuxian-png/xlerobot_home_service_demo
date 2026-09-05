import { useCallback, useEffect, useRef, useState } from 'react'

// Both workspaces use the same session. The server still owns the different
// mapping/operator permission checks; the browser never bypasses them.
export function useBaseTeleop(disabled: boolean, onError: (message: string) => void) {
  const [armed, updateArmed] = useState(false)
  const [connected, setConnected] = useState(false)
  const enabled = useRef(false)
  const disabledRef = useRef(disabled)
  const errorCallback = useRef(onError)
  disabledRef.current = disabled
  errorCallback.current = onError
  const socket = useRef<WebSocket | null>(null)
  const timer = useRef<number | null>(null)
  const command = useRef({ linear: 0, angular: 0 })

  const stop = useCallback(() => {
    enabled.current = false
    updateArmed(false)
    setConnected(false)
    if (timer.current !== null) window.clearInterval(timer.current)
    timer.current = null
    const ws = socket.current
    socket.current = null
    command.current = { linear: 0, angular: 0 }
    if (ws?.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify({ armed: false, linear: 0, angular: 0 }))
    }
    if (ws && ws.readyState < WebSocket.CLOSING) ws.close()
  }, [])

  const transmit = () => {
    if (enabled.current && socket.current?.readyState === WebSocket.OPEN) {
      socket.current.send(JSON.stringify({ armed: true, ...command.current }))
    }
  }
  const open = () => {
    if (socket.current) return
    const protocol = location.protocol === 'https:' ? 'wss' : 'ws'
    const ws = new WebSocket(`${protocol}://${location.host}/api/v1/teleop/base`)
    socket.current = ws
    const fail = (message: string) => {
      if (socket.current !== ws) return
      stop()
      errorCallback.current(message)
    }
    ws.onopen = () => {
      if (socket.current !== ws) return
      setConnected(true)
      transmit()
      timer.current = window.setInterval(transmit, 100)
    }
    ws.onmessage = event => {
      if (socket.current !== ws) return
      try {
        const message = JSON.parse(event.data)
        if (message.error) fail(String(message.error))
      } catch { fail('无法解析遥控服务响应，已结束遥控。') }
    }
    ws.onerror = () => fail('底盘遥控连接失败')
    ws.onclose = () => fail('遥控连接已断开，已停止发送；请重新开启遥控。')
  }
  const move = (linear: number, angular: number) => {
    if (!enabled.current || disabledRef.current) return
    if (!Number.isFinite(linear) || !Number.isFinite(angular)) { stop(); return }
    // Release sends zero on the same connection. Rapid re-press cannot race
    // a previous backend owner that is still closing its WebSocket.
    if (!socket.current && linear === 0 && angular === 0) return
    command.current = { linear, angular }
    open()
    transmit()
  }
  useEffect(() => { if (disabled) stop() }, [disabled, stop])
  useEffect(() => {
    const hidden = () => { if (document.hidden) stop() }
    window.addEventListener('blur', stop)
    document.addEventListener('visibilitychange', hidden)
    return () => {
      stop()
      window.removeEventListener('blur', stop)
      document.removeEventListener('visibilitychange', hidden)
    }
  }, [stop])
  return {
    armed, connected, move, stop,
    setArmed: (value: boolean) => {
      if (!value) { stop(); return }
      if (disabledRef.current) return
      enabled.current = true
      updateArmed(true)
    },
  }
}
