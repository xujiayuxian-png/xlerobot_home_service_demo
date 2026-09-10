import { useLanguage, type Message, LanguageToggle } from './i18n'
import { useEffect, useState } from 'react'
import { HeadCalibrationWorkspace } from './HeadCalibrationWorkspace'
import { ServoCalibrationWorkspace } from './ServoCalibrationWorkspace'
import { LiveCameraImage } from './LiveCameraImage'
import { HoverReport } from './HoverReport'

type Stage = 'servo' | 'leader' | 'head_camera' | 'right_handeye' | 'hover'
type Tab = Stage | 'results' | 'other'
type Row = { present?: boolean, quality_passed?: boolean, freshness: string,
  resume_issue?: string,
  missing: string[], metrics?: Record<string, number>, result_path?: string,
  provenance?: { summary?: Record<string, number>, report_path?: string } }
type Snapshot = {
  unit: string, active: string | null, runtime_matches_active: boolean,
  draft_complete: boolean, hardware_enabled: boolean, state_path: string,
  session: { stage: Stage | null, running: boolean, generation: string, error: string, log_path: string | null, startup_notice?: string },
  stages: Record<Stage, Row>, versions: string[],
}
const labels: Record<Tab, string> = {
  servo: '从臂与头部', leader: 'Leader 主臂', head_camera: '头部相机',
  right_handeye: '手眼标定', hover: '悬停精度', results: '结果管理', other: '其他标定说明',
}
const preparation: Record<Stage, string> = {
  servo: '准备手动摆臂空间。沿用已有舵机标定时，可直接查看结果，不必重新采集。',
  leader: '仅连接 Leader 主臂标定会话；独立保存，不替换从臂配置，也不阻塞视觉标定。',
  head_camera: '将 4×4 AprilTag 板平放桌面，头部相机需要看见完整 16 个 Tag。先启动设备会话，再在下方确认开始自动标定。',
  right_handeye: '将 60 mm Tag 23 固定在夹爪已约定的安装面，确保整个采样过程不松动。先准备到位，再开始 20 个拟合点和 6 个留出验证点。',
  hover: '保持手眼标定时的头部姿态，桌面板与 Tag 23 同时可见；至少露出 3 个不共线的桌面 Tag。目标为 Tag 中心在桌面板上方 200 mm，不是夹爪最低点。',
}

export async function workbenchRequest<T>(path: string, body?: object): Promise<T> {
  const response = await fetch(path, body === undefined ? undefined : {
    method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body),
  })
  if (!response.ok) throw new Error(await response.text())
  return response.json() as Promise<T>
}

export function CalibrationWorkbench() {
  const { t, s, language } = useLanguage()
  const [state, setState] = useState<Snapshot | null>(null)
  const [tab, setTab] = useState<Tab>('servo')
  const [busy, setBusy] = useState(false)
  const [error, setError] = useState<Message>('')
  const [connection, setConnection] = useState<Message>('')
  const [fresh, setFresh] = useState(false)
  const [log, setLog] = useState('')
  const [version, setVersion] = useState('')
  const [restore, setRestore] = useState('')
  const [components, setComponents] = useState<string[]>(['servo', 'head_camera', 'right_handeye'])
  const [preview, setPreview] = useState('')
  const refresh = async () => {
    setState(await workbenchRequest<Snapshot>('/workbench-api/status'))
  }
  useEffect(() => {
    let disposed = false
    let timer: ReturnType<typeof setTimeout>
    const poll = async () => {
      try {
        const value = await workbenchRequest<Snapshot>('/workbench-api/status')
        if (!disposed) { setState(value); setConnection('') }
      } catch (reason) { if (!disposed) setConnection(String(reason)) }
      if (!disposed) timer = setTimeout(poll, 1500)
    }
    void poll()
    return () => { disposed = true; clearTimeout(timer) }
  }, [])

  const command = async (operation: string, body: object = {}, question?: string) => {
    if (question && !window.confirm(question)) return
    setBusy(true); setError('')
    try {
      const result = await workbenchRequest<object>(`/workbench-api/${operation}`, { ...body, confirmed: true })
      if (operation === 'replace') setPreview(JSON.stringify(result, null, 2))
      await refresh()
    } catch (reason) { setError(String(reason)) }
    finally { setBusy(false) }
  }
  if (!state) return <main className="loading"><LanguageToggle /><p>{t("正在读取标定工作台…")}</p><small>{s(connection)}</small></main>
  const session = state.session
  const stage = tab !== 'results' && tab !== 'other' ? tab : null
  const row = stage ? state.stages[stage] : null
  const occupied = Boolean(session.stage)
  const blocked = busy || Boolean(connection)
  return <main className="shell calibration-workbench">
    <header className="app-header">
      <div><p className="eyebrow">XLEROBOT · CALIBRATION</p><h1>{t("机器人标定")}</h1>
        <p>{t("从臂 / 头部 → 头部相机 → 手眼标定 → 悬停验证 · Leader 独立")}</p></div>
      <div><strong>{state.unit}</strong><p>{t("当前生效：")}{state.active || t("尚未激活")}</p>
        <small>{state.runtime_matches_active ? t("运行配置与生效版本一致") : t("尚未生成有效运行配置")}</small></div>
      <LanguageToggle />
    </header>
    {!state.hardware_enabled && <div className="notice">{t("离线模式：可查看和管理已有结果，不连接设备。连接设备请用 tools/calibrate web --hardware 启动。")}</div>}
    {(error || connection) && <div className="notice" role="alert">{s(error || connection)}</div>}
    <nav className="operator-view-tabs" aria-label={t("标定步骤")}>
      {(Object.keys(labels) as Tab[]).map(key => <button key={key}
        className={tab === key ? 'active' : ''} onClick={() => { setTab(key); setFresh(false) }}>
        {t(labels[key])}</button>)}
    </nav>
    <section className="engineering-card">
      <strong>{t("设备会话：")}{session.stage ? t(labels[session.stage]) : t("未启动")}</strong>
      <span> · {session.stage ? session.running ? t("运行中") : t("进程已退出，请结束会话后重试") : t("浏览页面不会打开电机设备")}</span>
      {session.error && <p role="alert">{s(session.error)}</p>}
      {session.startup_notice && <div className="notice" role="alert"><strong>{s(session.startup_notice)}</strong></div>}
      <div className="workbench-actions">
        {session.stage === 'hover' && session.running && <button onClick={() => void workbenchRequest('/api/v1/hover/cancel', {})
          .catch(reason => setError(String(reason)))}>{t("停止自动验证")}</button>}
        {occupied && <button disabled={blocked} onClick={() => void command('stop', {},
          t("结束会话会停止当前动作并关闭驱动，机械臂可能失去保持力。请先支撑 / 放稳机械臂。确认结束？"))}>{t("结束设备会话")}</button>}
        {session.log_path && <button onClick={() => void workbenchRequest<{ text: string }>('/workbench-api/log')
          .then(value => setLog(value.text)).catch(reason => setError(String(reason)))}>{t("查看启动日志")}</button>}
      </div>
      {log && <details open><summary>{t("会话日志")}</summary><pre>{log}</pre></details>}
    </section>
    {stage && row && <section className="engineering-card">
      <h2>{t(labels[stage])}</h2><p>{t(preparation[stage])}</p>
      <p>{t("已有结果：")}{stage === 'hover' ? row.provenance?.report_path ? t("测量报告已保存") : t("尚无测量记录")
        : row.present ? row.quality_passed ? t("草稿质量门通过") : t("已保存，待检查") : t("暂无草稿")}
        {' · '}{row.freshness === 'current' ? stage === 'hover' ? t("测量所用标定未变化（不代表精度达标）") : t("前序版本一致")
          : row.freshness === 'needs_validation' ? t("前序关联尚未确认 / 已变化，需验证") : t("尚无本次验收记录")}</p>
      {row.metrics && <details><summary>{t("已有结果指标")}</summary><pre>{JSON.stringify(row.metrics, null, 2)}</pre></details>}
      {row.result_path && <p>{t("结果文件：")}<code>{row.result_path}</code></p>}
      {row.missing.length > 0 && <p className="notice">{t("需要先保存通过的草稿：")}{row.missing.map(n => t(labels[n as Tab])).join('、')}{t("。已有生效配置与草稿是分开的。")}</p>}
      {!occupied && <>
        {stage === 'hover' && <button className="primary" disabled={blocked || !state.hardware_enabled || row.missing.length > 0}
          onClick={() => void command('auto-hover', {},
            t("确认桌面板和 Tag 23 固定、三个目标附近及右臂和头部路径无障碍？将启动设备，自动准备、测三个点并回 ready；中途可停止。补偿仅建议，不自动应用。"))}>{t("一键启动并自动验证三点")}</button>}
        {row.resume_issue && <div className="notice" role="alert"><p>{s(row.resume_issue)}</p>
          <button disabled={blocked || !state.hardware_enabled || row.missing.length > 0}
            onClick={() => void command('start', { stage, fresh: true },
              t("旧采样将归档保留，开始新一轮设备会话；不会自动开始标定运动。确认继续？"))}>{t("归档旧记录，开始新一轮标定")}</button>
        </div>}
        {state.active && stage !== 'hover' && stage !== 'leader' && <button disabled={blocked}
          onClick={() => void command('reuse', { stage }, t("将当前生效版本中此项结果复制为前序草稿？不会重新测量；已有草稿会先归档。"))}>{t("使用当前生效结果作为草稿")}</button>}
        {stage !== 'hover' && <label><input type="checkbox" checked={fresh}
          onChange={e => setFresh(e.target.checked)} /> {t(" 归档旧采样，开始新一轮（不删除旧证据或当前生效配置）")}</label>}
        <div className="workbench-actions"><button disabled={blocked || !state.hardware_enabled || row.missing.length > 0 || Boolean(row.resume_issue && !fresh)}
          onClick={() => void command('start', { stage, fresh },
            t("启动所选设备会话？视觉会话会让头部 / 右臂保持当前位置；自动采样仍需在页面中另行开始。"))}>{stage === 'hover' ? t("仅启动设备（手动调试）") : t("启动此项设备会话")}</button></div>
      </>}
      {occupied && session.stage !== stage && <p>{t("当前运行的是“")}{t(labels[session.stage!])}{t("”。切换标签不会中断它；要更换设备会话，请先结束当前会话。")}</p>}
      {session.stage === stage && stage !== 'hover' && <button disabled={blocked || !session.running}
        onClick={() => void command('accept', {}, t("确认本项已完成？将保存为本机草稿并记录前序版本，不替换 Demo 当前配置。"))}>{t("确认完成并保存本项结果")}</button>}
    </section>}
    {tab === 'hover' && <HoverReport />}
    {/* Keep the active workspace mounted while browsing other tabs. Polling must
        never recreate the controller UI or interrupt an in-flight operation. */}
    {session.running && <div hidden={tab !== session.stage} key={session.generation}>
      {(session.stage === 'servo' || session.stage === 'leader') && <ServoCalibrationWorkspace
        unitId={state.unit} captureOnly onError={setError} restartBusy={busy}
        onRestart={() => void command('restart-servo', { stage: session.stage },
          t("将归档这轮舵机标定记录，并重新打开同一设备的标定会话。旧结果和生效标定保留，不自动上力或移动。确认重新开始？"))} />}
      {(session.stage === 'head_camera' || session.stage === 'right_handeye') && <HeadCalibrationWorkspace
        unitId={state.unit} handeye={session.stage === 'right_handeye'} onError={setError} />}
      {session.stage === 'hover' && <HoverWorkspace />}
    </div>}
    {tab === 'results' && <section className="engineering-card">
      <h2>{t("结果管理")}</h2><p>{t("本机保存目录：")}<code>{state.state_path}</code></p>
      <p>{t("更换配置前必须结束设备会话。应用后下次启动 Demo 才加载，不热切换运行中的坐标系。")}</p>
      <label>{t("新版本名称 ")}<input value={version} placeholder="my-calibration-v2"
        onChange={e => { setVersion(e.target.value); setPreview('') }} /></label>
      <fieldset><legend>{t("替换哪些草稿（其余保留当前生效值）")}</legend>
        {(['servo', 'head_camera', 'right_handeye'] as Stage[]).map(name => <label key={name}>
          <input type="checkbox" checked={components.includes(name)} onChange={e => {
            setComponents(current => e.target.checked ? [...current, name] : current.filter(n => n !== name)); setPreview('')
          }} />{t(labels[name])}</label>)}
      </fieldset>
      <p>{t("保留旧底盘 / 抓取补偿不意味着它们已重新验收。只替换上游参数时，应重新验证下游结果。")}</p>
      <div className="workbench-actions">
        <button disabled={blocked || occupied || !version || !state.active || !components.length}
          onClick={() => void command('replace', { version, components, preview: true })}>{t("预览替换")}</button>
        <button disabled={blocked || occupied || !preview || !version}
          onClick={() => void command('replace', { version, components }, t("应用选定草稿并保留其余现有值？旧版本保留，可恢复。"))}>{t("应用选定结果")}</button>
        {!state.active && <button disabled={blocked || occupied || !state.draft_complete || !version}
          onClick={() => void command('activate', { version }, t("激活完整首次标定结果并生成运行配置？"))}>{t("首次激活完整标定")}</button>}
      </div>
      {preview && <pre>{preview}</pre>}
      <label>{t("已保存版本 ")}<select value={restore} onChange={e => setRestore(e.target.value)}>
        <option value="">{t("选择要恢复的版本")}</option>{state.versions.map(v => <option key={v}>{v}</option>)}
      </select></label>
      <button disabled={blocked || occupied || !restore} onClick={() => void command('switch', { version: restore },
        t("恢复 {0} 并重新生成运行配置？", restore))}>{t("恢复所选版本")}</button>
      <p>{t("Leader 单独用于数采：")}<code>tools/act collect --hardware --leader-calibration PATH</code>{t("，不混入上述结构标定包。")}</p>
    </section>}
    {tab === 'other' && <section className="engineering-card"><h2>{t("其他标定说明")}</h2>
      <p>{t("底盘几何、雷达角度对齐和抓取对齐不放入这次网页必做流程。新机器人首次激活仍需按文档准备完整配置。")}</p>
      <p>{t("查看本地文档（无需联网）：")}<a href={`/workbench-docs/calibration-followup.md?lang=${language}`}
        target="_blank" rel="noreferrer">{t("底盘 / 雷达 / 抓取对齐")}</a> · <a
        href={`/workbench-docs/calibration-versions.md?lang=${language}`}
        target="_blank" rel="noreferrer">{t("配置替换与恢复")}</a></p>
    </section>}
  </main>
}

type HoverState = { phase: string, message: string, busy: boolean, motion_unconfirmed: boolean,
  ready?: boolean, auto_progress?: { completed: number, total: number, target: string } | null,
  plan: { id: string, target: string, goal: number[], minimum_table_clearance_mm: number, hardware_executed: boolean } | null,
  result: { summary: Record<string, number | number[]>, frames: number } | null }
function HoverWorkspace() {
  const { t, s } = useLanguage()
  const [state, setState] = useState<HoverState | null>(null)
  const [target, setTarget] = useState('center')
  const [busy, setBusy] = useState(false)
  const [error, setError] = useState<Message>('')
  const [connection, setConnection] = useState<Message>('')
  const [starting, setStarting] = useState(true)
  useEffect(() => {
    let disposed = false
    let connected = false
    const startedAt = Date.now()
    let timer: ReturnType<typeof setTimeout>
    const poll = async () => {
      try {
        const response = await fetch('/api/v1/hover/status')
        if (!response.ok) {
          if (response.status === 503 && !connected && Date.now() - startedAt < 30000) {
            if (!disposed) { setStarting(true); setConnection('') }
          } else throw new Error("悬停服务暂时无法连接，正在重试；持续失败请查看设备会话日志。")
        } else {
          const value = await response.json() as HoverState
          connected = true
          if (!disposed) { setState(value); setStarting(false); setConnection('') }
        }
      } catch (reason) {
        if (!disposed) { setStarting(false); setConnection(String(reason)) }
      }
      if (!disposed) timer = setTimeout(poll, 700)
    }
    void poll()
    return () => { disposed = true; clearTimeout(timer) }
  }, [])
  const run = async (operation: string) => {
    if (operation === 'auto' && !window.confirm(t("确认标定板固定，头部、右臂以及三个目标附近无障碍？将自动测三个点并回 ready；补偿仅建议，不自动应用。"))) return
    if (operation === 'prepare' && !window.confirm(t("将低头并移动右臂到手眼观测姿态。确认头部和右臂周围无障碍？"))) return
    if (operation === 'execute' && !window.confirm(t("确认预览路径上无障碍？将用 12 秒移动右臂到高位目标，桌面以外的障碍需要现场检查。"))) return
    setBusy(true); setError('')
    try {
      const value = await workbenchRequest<HoverState>(`/api/v1/hover/${operation}`, {
        target, plan_id: state?.plan?.id, confirmed: ['auto', 'execute', 'prepare'].includes(operation),
      })
      if (operation !== 'auto') setState(value)
    } catch (reason) { setError(String(reason)) }
    finally { setBusy(false) }
  }
  const blocked = busy || starting || Boolean(connection) || state?.busy || state?.motion_unconfirmed
  const metricLabels: Record<string, string> = { planar_error_mm: t("平面误差 mm"),
    height_shortfall_mm: t("高度不足 mm（正值代表偏低）"), height_mm: t("测量高度 mm"),
    error_3d_mm: t("三维位置误差 mm"), visual_fk_closure_mm: t("视觉–FK 差异 mm"),
    joint_tracking_error_deg: t("五关节到位偏差 °") }
  return <section className="engineering-card">
    <h2>{t("自动悬停验证")}</h2><p>{s(state?.message) || t("等待悬停服务…")}</p>
    {starting && <p role="status">{t("正在启动悬停服务，连接成功后会自动更新，无需刷新页面。")}</p>}
    {connection && <p role="alert">{s(connection)}</p>}
    <p>{t("一次确认：准备观测姿态 → 中心 / X +30 / X −30 → 每点 20 帧 → 回 ready → 结果与建议。")}</p>
    <div className="workbench-actions"><button className="primary" disabled={!state?.ready || Boolean(blocked)} onClick={() => void run('auto')}>{t("自动运行三点验证")}</button>
      <button disabled={!state?.busy && !state?.motion_unconfirmed} onClick={() => void workbenchRequest('/api/v1/hover/cancel', {})
        .catch(reason => setError(String(reason)))}>{t("停止自动验证 / 移动")}</button></div>
    {state?.auto_progress && <p role="status">{t("已测 ")}{state.auto_progress.completed} / {state.auto_progress.total} {t(" 点 · 当前阶段：")}{({ prepare: t("准备观测姿态"), center: t("中心点"), right: t("右侧点"), left: t("左侧点"), return_ready: t("返回准备姿态") } as Record<string, string>)[state.auto_progress.target] || state.auto_progress.target}</p>}
    {error && <p role="alert">{s(error)}</p>}
    <LiveCameraImage className="workbench-camera" src="/api/v1/cameras/head/stream" alt={t("头部相机实时画面：检查桌面板与夹爪 Tag 23")} />
    <details><summary>{t("单点调试（可选）")}</summary><label>{t("目标点 ")}<select value={target} disabled={Boolean(blocked)} onChange={e => setTarget(e.target.value)}>
      <option value="center">{t("板中心 · 上方 200 mm")}</option><option value="right">{t("板 X +30 mm · 上方 200 mm")}</option>
      <option value="left">{t("板 X −30 mm · 上方 200 mm")}</option></select></label>
    <div className="workbench-actions">
      <button disabled={!state || Boolean(blocked)} onClick={() => void run('prepare')}>{t("准备观测姿态（头部 / 右臂）")}</button>
      <button disabled={!state || Boolean(blocked)} onClick={() => void run('preview')}>{t("预览目标（不移动）")}</button>
      <button disabled={!state?.plan || state.plan.target !== target || Boolean(blocked) || state.plan.hardware_executed}
        onClick={() => void run('execute')}>{t("确认并移动")}</button>
      <button disabled={!state?.plan?.hardware_executed || Boolean(blocked)} onClick={() => void run('measure')}>{t("测量并保存 20 帧")}</button>
      <button disabled={!state?.motion_unconfirmed} onClick={() => void workbenchRequest('/api/v1/hover/cancel', {})
        .catch(reason => setError(String(reason)))}>{t("停止移动")}</button>
    </div>
    {state?.plan && <p>{t("路径最小桌面包络间距：")}{state.plan.minimum_table_clearance_mm.toFixed(1)} {t(" mm；目标关节： ")}{state.plan.goal.map(q => q.toFixed(3)).join(', ')} rad</p>}
    {state?.result && <table><thead><tr><th>{t("指标")}</th><th>{t("结果")}</th></tr></thead><tbody>
      {Object.entries(state.result.summary).map(([name, value]) => <tr key={name}><td>{t(metricLabels[name]) || name}</td>
        <td>{Array.isArray(value) ? value.map(n => n.toFixed(2)).join(', ') : value.toFixed(2)}</td></tr>)}
    </tbody></table>}</details>
    <p>{t("结果自动保存。20 帧是一处悬停的静止观测，不代表重复到位精度；同相机视觉测量也不是独立尺量真值。 完成三点后给出总偏移建议，但不自动应用，也不因控制器报告成功而判断精度合格。")}</p>
  </section>
}
