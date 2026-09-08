import { useEffect, useRef, useState } from 'react'
import { api, type HeadCalibrationState } from './api'

const phases: Record<string, string> = {
  IDLE: '准备就绪', MOVING: '头部移动中', WAITING: '等待清晰、稳定的标定板',
  CAPTURING: '保存样本', SOLVING: '正在求解', PAUSED: '已暂停',
  COMPLETED: '标定完成', ERROR: '需要处理后继续',
}
const poseLabels: Record<string, string> = {
  pending: '待采集', moving: '移动中', waiting: '等待识别', captured: '已采集', skipped: '待补采',
}
const metricLabels: Record<string, string> = {
  translation_rmse_mm: '平移 RMS · mm', translation_p95_mm: '平移 p95 · mm',
  translation_max_mm: '最大平移误差 · mm', rotation_rmse_deg: '旋转 RMS · °',
  reprojection_rmse_px: '重投影 RMS · px', sample_count: '有效样本数',
}

export function HeadCalibrationWorkspace({ unitId, onError }: {
  unitId: string, onError: (message: string) => void,
}) {
  const [state, setState] = useState<HeadCalibrationState | null>(null)
  const [confirmed, setConfirmed] = useState(false)
  const [busy, setBusy] = useState('')
  const [awaitingStart, setAwaitingStart] = useState(false)
  const [readError, setReadError] = useState('')
  const [actionError, setActionError] = useState('')
  const [notice, setNotice] = useState('')
  const [receivedAt, setReceivedAt] = useState(0)
  const [now, setNow] = useState(Date.now())
  const busyRef = useRef(false)
  const mounted = useRef(false)
  const onErrorRef = useRef(onError)
  onErrorRef.current = onError

  useEffect(() => {
    mounted.current = true
    let disposed = false
    let timer: ReturnType<typeof setTimeout>
    const clock = setInterval(() => setNow(Date.now()), 1000)
    const poll = async () => {
      try {
        const value = await api.headCalibrationStatus()
        if (!disposed) {
          setState(value)
          setReadError('')
          setReceivedAt(Date.now())
          if (value.request_inflight || value.running || value.phase !== 'IDLE') setAwaitingStart(false)
        }
      } catch (reason) {
        if (!disposed) setReadError(String(reason))
      }
      if (!disposed) timer = setTimeout(poll, 500)
    }
    void poll()
    return () => { disposed = true; mounted.current = false; clearTimeout(timer); clearInterval(clock) }
  }, [unitId])

  const live = Boolean(state?.available && state.state_fresh && !readError
    && receivedAt > 0 && now - receivedAt < 4000)
  const ready = live && state?.action_ready && state.unit_id === unitId
  const running = Boolean(state?.running || state?.request_inflight || awaitingStart)
  const complete = state?.phase === 'COMPLETED' && state.quality_passed
  const targetVisible = Boolean(live && state?.target.fresh && state.target.accepted)
  const count = state?.sample_count ?? 0
  const required = state?.target_sample_count || 12
  const resume = Boolean(state && (count > 0 || state.phase === 'PAUSED' || state.phase === 'ERROR'))
  const startLabel = complete ? '本次标定已完成' : resume ? '继续自动采集 / 补采' : '开始自动标定'

  const command = async (operation: 'start' | 'pause' | 'preview') => {
    if (busyRef.current) return
    busyRef.current = true
    setBusy(operation)
    setActionError('')
    setNotice('')
    try {
      const result = operation === 'start' ? await api.startHeadCalibration(unitId)
        : operation === 'pause' ? await api.pauseHeadCalibration()
          : await api.moveCalibrationPose(0)
      if (!mounted.current) return
      setNotice(result.message)
      if (operation === 'start') { setAwaitingStart(true); setConfirmed(false) }
    } catch (reason) {
      if (mounted.current) {
        const message = String(reason)
        setActionError(message)
        onErrorRef.current(message)
      }
    } finally {
      busyRef.current = false
      if (mounted.current) setBusy('')
    }
  }

  return <section className="engineering-card head-calibration">
    <div className="head-calibration-heading">
      <div><p className="section-label">HEAD CAMERA · {unitId}</p>
        <h2>头部相机自动标定</h2>
        <p>放好整板，剩下的交给机器人：转头 → 等待稳定 → 采样 → 求解。</p></div>
      <span className={`head-phase ${complete ? 'complete' : ''}`} role="status">
        {!live ? '等待实时状态' : phases[state?.phase || ''] || state?.phase}
      </span>
    </div>
    <div className="head-calibration-layout">
      <div className="head-calibration-view">
        <figure className="head-camera-preview">
          <img src="/api/v1/cameras/detection/stream" alt="D455 标定板实时画面与 AprilTag 检测框" />
          <figcaption>AprilTag 36h11 · 4 × 4 · ID 0–15 · 单 Tag 40 mm</figcaption>
        </figure>
        <div className="head-visual-metrics" aria-label="标定板识别状态">
          <div className={targetVisible ? 'good' : 'waiting'}>
            <small>标定板</small><strong>{targetVisible ? '整板识别通过' : '等待整板清晰可见'}</strong></div>
          <div><small>识别 Tag</small><strong>{live && state?.target.fresh ? state.target.tag_count ?? 0 : '—'} / 16</strong></div>
          <div><small>重投影误差 · px</small><strong>{live && state?.target.fresh
            && state.target.reprojection_rmse_px != null
            ? state.target.reprojection_rmse_px.toFixed(2) : '—'}</strong></div>
        </div>
        {state?.target.detail && <details className="head-diagnostic"><summary>识别诊断</summary>
          <p>{state.target.detail}</p>
          <p>{state.target.fresh ? '观测为最新一帧' : '观测已过期或尚未收到；旧画面不能用于采样。'}</p></details>}
      </div>
      <div className="head-calibration-control">
        <p className="section-label">自动采集</p>
        <div className="head-sample-count"><strong>{count}</strong><span>有效样本 / 至少 {required} 个</span></div>
        <progress max={Math.max(required, count)} value={count} aria-label="有效样本进度" />
        <p className="head-run-message" aria-live="polite">{complete
          ? `已保存 ${count} 个有效样本，求解与质量检查通过；结果仅写入草稿。`
          : state?.message || '连接标定服务后，可先调整头部到看桌面的预览位置。'}</p>
        <label className="head-motion-confirm">
          <input type="checkbox" checked={confirmed} disabled={running || Boolean(busy)}
            onChange={event => setConfirmed(event.target.checked)} />
          <span>标定板与底盘已固定，头部周围无遮挡；允许本次头部动作。</span>
        </label>
        <button disabled={!ready || !confirmed || running || Boolean(busy)}
          onClick={() => void command('preview')}>头部低头到预览位（0 / 0.8 rad）</button>
        <div className="button-row head-run-buttons">
          <button className="primary" disabled={!ready || !confirmed || running || Boolean(busy) || complete}
            onClick={() => void command('start')}>{busy === 'start' ? '正在启动…'
              : running ? state?.phase === 'SOLVING' ? '正在求解…' : '自动采集中…'
                : startLabel}</button>
          <button disabled={!running || Boolean(busy)} onClick={() => void command('pause')}>
            {busy === 'pause' ? '正在请求暂停…' : '暂停并保留样本'}</button>
        </div>
        {!ready && <p className="head-warning" role="alert">{readError
          ? `状态连接中断，暂不能启动。${readError}` : '自动标定服务未就绪或状态已过期，请等待实时状态恢复。'}</p>}
        {actionError && <p className="head-warning" role="alert">{actionError}</p>}
        {notice && <p className="hint" aria-live="polite">{notice}</p>}
        <p className="hint">启动网页、刷新或重连都不会自动运动。暂停后头部保持当前位置，继续时只补未完成的姿态。</p>
      </div>
    </div>
    <div className="head-pose-section">
      <div className="head-pose-heading"><h3>采集姿态</h3>
        <p>{state?.pose_count ?? 0} 个预设姿态 · 水平 ±0.30 rad · 低头 0.60–1.00 rad</p></div>
      <ol className="head-pose-grid" aria-label="预设姿态采集进度">
        {(state?.pose_states || []).map((pose, index) => <li key={index}
          className={`head-pose ${pose}`} aria-current={state?.pose_index === index ? 'step' : undefined}>
          <span className="head-pose-number">{String(index + 1).padStart(2, '0')}</span>
          <strong>{complete && pose === 'skipped' ? '已跳过' : poseLabels[pose] || pose}</strong>
          <small>{state?.pose_pan[index]?.toFixed(2)} / {state?.pose_tilt[index]?.toFixed(2)} rad</small>
        </li>)}
      </ol>
      <p className="hint">{complete
        ? '本次样本数量、姿态覆盖和残差均已通过，无需补齐全部预设姿态。拟合残差不等同于独立实测精度。'
        : '短暂漏检会等待后重试；待补采姿态可在暂停或失败后继续。全程不要移动标定板或底盘。'}</p>
    </div>
    {(state?.result_uri || state?.phase === 'COMPLETED' || state?.phase === 'ERROR') &&
      <section className="head-calibration-result" aria-label="标定结果">
        <h3>{state.quality_passed ? '质量检查通过，结果已保存' : '采集 / 质量检查尚未通过'}</h3>
        <dl className="head-result-metrics">{Object.entries(state.metrics || {}).map(([name, value]) =>
          <div key={name}><dt>{metricLabels[name] || name}</dt><dd>{value == null ? '—' : value.toFixed(3)}</dd></div>)}</dl>
        {state.result_uri && <p className="head-result-path">结果：<code>{state.result_uri}</code></p>}
        <p>不会自动替换当前生效标定，也不会自动运行 Demo。</p>
      </section>}
    <details className="head-restart-help"><summary>板挪过位置了 / 需要从头重来？</summary>
      <p>一组样本必须使用同一个固定标定板位置。只要移动过标定板或底盘，就不要继续原会话。</p>
      <p>先暂停并停止当前标定工具，再启动以下命令。旧采样会归档保留，不会删除。</p>
      <code>./tools/calibrate capture head-camera --hardware --fresh</code>
    </details>
  </section>
}
