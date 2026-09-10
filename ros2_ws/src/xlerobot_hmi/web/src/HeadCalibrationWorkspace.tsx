import { useLanguage, type Message } from './i18n'
import { useEffect, useRef, useState } from 'react'
import { api, type HeadCalibrationState } from './api'
import { LiveCameraImage } from './LiveCameraImage'

const phases: Record<string, string> = {
  IDLE: '准备就绪', MOVING: '头部移动中', WAITING: '等待清晰、稳定的标定板',
  CAPTURING: '保存样本', SOLVING: '正在求解', PAUSED: '已暂停',
  COMPLETED: '标定完成', ERROR: '需要处理后继续',
  VALIDATING: '独立留出验证中',
  RETURNING: '结果已保存 · 正在回 ready',
}
const poseLabels: Record<string, string> = {
  pending: '待采集', moving: '移动中', waiting: '等待识别', captured: '已采集', skipped: '待补采',
}
const metricLabels: Record<string, string> = {
  translation_rmse_mm: '平移 RMS · mm', translation_p95_mm: '平移 p95 · mm',
  translation_max_mm: '最大平移误差 · mm', rotation_rmse_deg: '旋转 RMS · °',
  reprojection_rmse_px: '重投影 RMS · px', sample_count: '有效样本数',
}

export function HeadCalibrationWorkspace({ unitId, onError, handeye = false }: {
  unitId: string, onError: (message: Message) => void, handeye?: boolean,
}) {
  const { t, s } = useLanguage()
  const [state, setState] = useState<HeadCalibrationState | null>(null)
  const [confirmed, setConfirmed] = useState(false)
  const [busy, setBusy] = useState('')
  const [awaitingStart, setAwaitingStart] = useState(false)
  const [readError, setReadError] = useState<Message>('')
  const [actionError, setActionError] = useState<Message>('')
  const [notice, setNotice] = useState<Message>('')
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
        const value = await (handeye ? api.handeyeCalibrationStatus() : api.headCalibrationStatus())
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
  }, [unitId, handeye])

  const live = Boolean(state?.available && state.state_fresh && !readError
    && receivedAt > 0 && now - receivedAt < 4000)
  const ready = live && state?.action_ready && state.unit_id === unitId
  const running = Boolean(state?.running || state?.request_inflight || awaitingStart)
  const complete = state?.phase === 'COMPLETED' && state.quality_passed
  const targetVisible = Boolean(live && state?.target.fresh && state.target.accepted)
  const count = state?.sample_count ?? 0
  const required = state?.target_sample_count || (handeye ? 26 : 12)
  const resume = Boolean(state && (count > 0 || state.phase === 'PAUSED' || state.phase === 'ERROR'))
  const startLabel = complete ? t("本次标定已完成") : resume ? t("继续自动采集 / 补采") : t("开始自动标定")

  const command = async (operation: 'start' | 'pause' | 'preview' | 'reset') => {
    if (busyRef.current) return
    if (operation === 'reset' && !window.confirm(t("归档本轮样本和报告，开始新的标定会话？已有草稿和生效标定不变，不会运动或释放扭矩。"))) return
    busyRef.current = true
    setBusy(operation)
    setActionError('')
    setNotice('')
    try {
      const result = operation === 'reset' ? await api.resetVisualCalibration(unitId, handeye)
        : operation === 'start' ? await (handeye ? api.startHandeyeCalibration(unitId) : api.startHeadCalibration(unitId))
        : operation === 'pause' ? await (handeye ? api.pauseHandeyeCalibration() : api.pauseHeadCalibration())
          : await api.moveCalibrationPose(0)
      if (!mounted.current) return
      setNotice(result.message)
      if (operation === 'start') { setAwaitingStart(true); setConfirmed(false) }
      if (operation === 'reset') {
        setConfirmed(false)
        setAwaitingStart(false)
        setState(null)
        setReceivedAt(0)
      }
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
      <div><p className="section-label">{handeye ? 'RIGHT HAND–EYE' : 'HEAD CAMERA'} · {unitId}</p>
        <h2>{handeye ? t("右臂手眼自动标定与验证") : t("头部相机自动标定")}</h2>
        <p>{handeye ? t("成功保存后，右臂和头部回到 ready；夹爪保持不变。") : t("成功保存后头部回到 ready，不操作机械臂。")}{t("暂停或失败时不会自动回位。")}</p>
        <p>{handeye ? t("固定夹爪 Tag 23：20 姿态拟合 → 冻结参数 → 6 个独立姿态验证。") : t("放好整板，剩下的交给机器人：转头 → 等待稳定 → 采样 → 求解。")}</p></div>
      <span className={`head-phase ${complete ? 'complete' : ''}`} role="status">
        {!live ? t("等待实时状态") : handeye && state?.phase === 'MOVING' ? t("右臂 / 头部移动中")
          : handeye && state?.phase === 'WAITING' ? t("等待稳定的 Tag 23") : t(phases[state?.phase || '']) || state?.phase}
      </span>
    </div>
    <div className="head-calibration-layout">
      <div className="head-calibration-view">
        <figure className="head-camera-preview">
          <LiveCameraImage src="/api/v1/cameras/detection/stream" ready={state !== null}
            alt={t("D455 标定板实时画面与 AprilTag 检测框")} />
          <figcaption>{handeye ? t("AprilTag 36h11 · Tag 23 · 黑色外边框边长 60 mm · 固定在右夹爪固定侧") : t("AprilTag 36h11 · 4 × 4 · ID 0–15 · 单 Tag 40 mm")}</figcaption>
        </figure>
        <div className="head-visual-metrics" aria-label={t("标定板识别状态")}>
          <div className={targetVisible ? 'good' : 'waiting'}>
            <small>{t("标定板")}</small><strong>{targetVisible ? handeye ? t("Tag 23 识别通过") : t("整板识别通过")
              : handeye ? t("等待 Tag 23 清晰可见") : t("等待整板清晰可见")}</strong></div>
          <div><small>{t("识别 Tag")}</small><strong>{live && state?.target.fresh ? state.target.tag_count ?? 0 : '—'} / {handeye ? 1 : 16}</strong></div>
          <div><small>{t("重投影误差 · px")}</small><strong>{live && state?.target.fresh
            && state.target.reprojection_rmse_px != null
            ? state.target.reprojection_rmse_px.toFixed(2) : '—'}</strong></div>
        </div>
        {state?.target.detail && <details className="head-diagnostic"><summary>{t("识别诊断")}</summary>
          <p>{s(state.target.detail)}</p>
          <p>{state.target.fresh ? t("观测为最新一帧") : t("观测已过期或尚未收到；旧画面不能用于采样。")}</p></details>}
      </div>
      <div className="head-calibration-control">
        <p className="section-label">{t("自动采集")}</p>
        <div className="head-sample-count"><strong>{count}</strong><span>{t("有效样本 / 至少 ")}{required} {t(" 个")}</span></div>
        <progress max={Math.max(required, count)} value={count} aria-label={t("有效样本进度")} />
        <p className="head-run-message" aria-live="polite">{complete
          ? t("已保存 {0} 个有效样本，求解与质量检查通过；结果仅写入草稿。", count)
          : s(state?.message) || (handeye ? t("启动前确认 Tag 安装牢固、右臂活动范围空旷。") : t("连接标定服务后，可先调整头部到看桌面的预览位置。"))}</p>
        <label className="head-motion-confirm">
          <input type="checkbox" checked={confirmed} disabled={running || Boolean(busy)}
            onChange={event => setConfirmed(event.target.checked)} />
          <span>{handeye ? t("Tag 23 与底盘已固定，右臂周围无遮挡且有人看护；允许本次右臂与头部动作。")
            : t("标定板与底盘已固定，头部周围无遮挡；允许本次头部动作。")}</span>
        </label>
        <button disabled={!ready || !confirmed || running || Boolean(busy)}
          onClick={() => void command('preview')}>{handeye ? t("右臂与头部到第 1 个采集位（会运动）") : t("头部低头到预览位（0 / 0.8 rad）")}</button>
        <div className="button-row head-run-buttons">
          <button className="primary" disabled={!ready || !confirmed || running || Boolean(busy) || complete}
            onClick={() => void command('start')}>{busy === 'start' ? t("正在启动…")
              : running ? state?.phase === 'SOLVING' ? t("正在求解…") : t("自动采集中…")
                : startLabel}</button>
          <button disabled={!running || Boolean(busy)} onClick={() => void command('pause')}>
            {busy === 'pause' ? t("正在请求暂停…") : t("暂停并保留样本")}</button>
        </div>
        {!ready && <p className="head-warning" role="alert">{readError
          ? t("状态连接中断，暂不能启动。{0}", readError) : t("自动标定服务未就绪或状态已过期，请等待实时状态恢复。")}</p>}
        {actionError && <p className="head-warning" role="alert">{s(actionError)}</p>}
        {notice && <p className="hint" aria-live="polite">{s(notice)}</p>}
        <p className="hint">{t("打开网页、刷新或重连不会发起运动。暂停后保持当前位置，继续时只补未完成的姿态。")}</p>
        {handeye && <p className="hint">{t("采样时头部固定在 0 / 0.796 rad，不开合夹爪，不移动底盘。留出验证测的是视觉–FK 闭合误差，不代表绝对抓取精度。")}</p>}
      </div>
    </div>
    <div className="head-pose-section">
      <div className="head-pose-heading"><h3>{t("采集姿态")}</h3>
        <p>{state?.pose_count ?? 0} {t(" 个预设姿态 · ")}{handeye ? t("前 20 个拟合，后 6 个只验证") : t("水平 ±0.30 rad · 低头 0.60–1.00 rad")}</p></div>
      <ol className="head-pose-grid" aria-label={t("预设姿态采集进度")}>
        {(state?.pose_states || []).map((pose, index) => <li key={index}
          className={`head-pose ${pose}`} aria-current={state?.pose_index === index ? 'step' : undefined}>
          <span className="head-pose-number">{String(index + 1).padStart(2, '0')}</span>
          <strong>{complete && pose === 'skipped' ? t("已跳过") : t(poseLabels[pose]) || pose}</strong>
          <small>{handeye ? state?.pose_roles?.[index] === 'validation' ? t("独立验证") : t("拟合")
            : `${state?.pose_pan[index]?.toFixed(2)} / ${state?.pose_tilt[index]?.toFixed(2)} rad`}</small>
          {handeye && state?.pose_messages?.[index] && <small title={s(state.pose_messages[index])}>{pose === 'skipped' ? s(state.pose_messages[index]) : ''}</small>}
        </li>)}
      </ol>
      <p className="hint">{complete
        ? handeye ? t("拟合和独立验证均通过；这是视觉/FK 一致性验证，不等同于夹爪尖端绝对精度。")
          : t("本次样本数量、姿态覆盖和残差均已通过，无需补齐全部预设姿态。拟合残差不等同于独立实测精度。")
        : handeye ? t("Tag 随夹爪移动，安装不能松动。拟合未通过不会进入验证；验证数据不会用于重新拟合。")
          : t("短暂漏检会等待后重试；待补采姿态可在暂停或失败后继续。全程不要移动标定板或底盘。")}</p>
    </div>
    {state && (state.result_uri || Object.keys(state.metrics || {}).length > 0 || state.phase === 'COMPLETED' || state.phase === 'ERROR') &&
      <section className="head-calibration-result" aria-label={t("标定结果")}>
        <h3>{state.quality_passed ? t("质量检查通过，结果已保存") : t("采集 / 质量检查尚未通过")}</h3>
        {(handeye ? ['fit', 'validation'] : ['']).map(group => <div key={group}>
          {group && <h4>{group === 'fit' ? t("拟合残差 · 20 姿态") : t("独立验证 · 6 姿态（不重新拟合）")}</h4>}
          <dl className="head-result-metrics">{Object.entries(state.metrics || {})
            .filter(([name]) => !group || name.startsWith(`${group}.`)).map(([name, value]) =>
              <div key={name}><dt>{t(metricLabels[group ? name.slice(group.length + 1) : name]) || name}</dt><dd>{value == null ? '—' : value.toFixed(3)}</dd></div>)}</dl>
        </div>)}
        {handeye && <p>{t("两组分别检查：平移 RMS < 10 mm、p95 < 15 mm，旋转 RMS < 5°。最大误差只报告。")}</p>}
        {handeye && Object.keys(state.metrics || {}).some(name => name.startsWith('heldout_poses.')) &&
          <table className="handeye-errors" aria-label={t("独立验证逐姿态误差")}><thead><tr><th>{t("验证姿态")}</th><th>{t("平移 · mm")}</th><th>{t("旋转 · °")}</th></tr></thead>
            <tbody>{(state.pose_roles || []).map((role, index) => role === 'validation' &&
              <tr key={index}><td>{index + 1}</td>
                <td>{state.metrics[`heldout_poses.${index + 1}.translation_mm`]?.toFixed(2) ?? '—'}</td>
                <td>{state.metrics[`heldout_poses.${index + 1}.rotation_deg`]?.toFixed(2) ?? '—'}</td></tr>)}</tbody></table>}
        {state.result_uri && <p className="head-result-path">{t("结果：")}<code>{state.result_uri}</code></p>}
        {state.report_uri && <p className="head-result-path">{t("逐姿态验证报告：")}<code>{state.report_uri}</code></p>}
        <p>{t("不会自动替换当前生效标定，也不会自动运行 Demo。")}</p>
      </section>}
    <section className="head-restart-help"><h3>{t("重新标定")}</h3>
      <p>{handeye ? t("Tag 必须保持同一安装位置；重新安装 Tag、移动底盘、修改头部或舵机标定后，请新建会话。")
        : t("一组样本必须使用同一个固定标定板位置。只要移动过标定板或底盘，就不要继续原会话。")}</p>
      <p>{t("归档本轮样本、拟合与验证报告，然后回到待开始状态。不会运动或释放扭矩；已有草稿和生效标定不变。运行中请先暂停。")}</p>
      <button disabled={!ready || running || Boolean(busy)} onClick={() => void command('reset')}>
        {busy === 'reset' ? t("正在归档…") : t("归档并重新标定")}</button>
      <p>{t("若修改了舵机或头相机标定、采样配置，仍需重新启动工具加载新配置。")}</p>
    </section>
  </section>
}
