import { useLanguage, msg, type Message, LanguageToggle } from './i18n'
import { FormEvent, useEffect, useMemo, useRef, useState } from 'react'
import { api, type CalibrationCoverage, type SiteSummary } from './api'
import { JoystickPad } from './JoystickPad'
import { ServoCalibrationWorkspace } from './ServoCalibrationWorkspace'
import { HeadCalibrationWorkspace } from './HeadCalibrationWorkspace'
import { useBaseTeleop } from './useBaseTeleop'
import {
  activeCameraPerception, activePersonTarget, taskShowsNavigationPath,
} from './observability'
import { upsertTaskHistory } from './taskHistory'
import type {
  Bootstrap, CollectionEpisode, CollectionState, Health, MappingState, NamedPlace,
  PerceptionState, Task,
} from './types'

const workspaceNames: Record<string, string> = {
  operator: '日常运行',
  mapping: '现场配置',
  calibration: '标定工作台',
  collection: '数据采集',
}

const voiceNames: Record<string, string> = {
  WAITING_FOR_WAKE: '等待唤醒',
  RECORDING: '正在录音',
  TRANSCRIBING: '正在识别',
  PARSING: '理解指令',
  EXECUTING_TASK: '执行任务',
  DISABLED: '未启用',
  UNKNOWN: '等待状态',
}

const placeNames: Record<string, string> = {
  table: '桌边',
}

const placePalette = [
  { stroke: '#ffca63', fill: 'rgba(255, 202, 99, .28)' },
  { stroke: '#c99cff', fill: 'rgba(201, 156, 255, .28)' },
  { stroke: '#58d9ff', fill: 'rgba(88, 217, 255, .28)' },
  { stroke: '#ff8f70', fill: 'rgba(255, 143, 112, .28)' },
  { stroke: '#8de36f', fill: 'rgba(141, 227, 111, .28)' },
]

const capabilityNames: Record<string, string> = {
  auto_localize: '确认定位',
  navigate_to_named_place: '前往桌边',
  detect_object: '识别物体',
  grasp_object: '抓取物体',
  scan_for_person: '寻找人员',
  approach_target: '接近人员',
  speak_text: '语音提示',
  handover_object: '递送物体',
}

export function preferNewestTask(
  current: Task | null, incoming: Task | null,
): Task | null {
  if (!current || !incoming || current.task_id !== incoming.task_id) {
    return incoming
  }
  return incoming.updated_at >= current.updated_at ? incoming : current
}

export function App() {
  const { t, s } = useLanguage()
  const [bootstrap, setBootstrap] = useState<Bootstrap | null>(null)
  const [health, setHealth] = useState<Health | null>(null)
  const [task, setTask] = useState<Task | null>(null)
  const [history, setHistory] = useState<Task[]>([])
  const [workspace, setWorkspace] = useState('operator')
  const [mapping, setMapping] = useState<MappingState | null>(null)
  const [perception, setPerception] = useState<PerceptionState | null>(null)
  const [voiceTranscript, setVoiceTranscript] = useState('')
  const [collection, setCollection] = useState<CollectionState | null>(null)
  const [operatorView, setOperatorView] = useState<'demo' | 'manual'>('demo')
  const [driveStopLatched, setDriveStopLatched] = useState<boolean | null>(null)
  const [stopBusy, setStopBusy] = useState(false)
  const [error, setError] = useState<Message>('')

  const refresh = async () => {
    const [boot, healthState, tasks] = await Promise.all([
      api.bootstrap(), api.health(), api.recentTasks(),
    ])
    setBootstrap(boot)
    setHealth(healthState)
    setTask(boot.active_task)
    setMapping(boot.mapping)
    setPerception(boot.perception)
    setVoiceTranscript(boot.voice_transcript || '')
    setCollection(boot.collection)
    setDriveStopLatched(boot.drive_stop_latched)
    setHistory(tasks.tasks)
    setWorkspace(current => boot.available_workspaces.includes(current)
      ? current : (boot.available_workspaces[0] || ''))
  }

  useEffect(() => {
    refresh().catch(reason => setError(String(reason)))
    const events = new EventSource('/api/v1/events')
    events.onmessage = event => {
      const update = JSON.parse(event.data)
      if (update.type === 'snapshot') {
        setHealth(update.data.health)
        setTask(update.data.task)
        if (update.data.history) setHistory(update.data.history)
        setMapping(update.data.mapping)
        setPerception(update.data.perception)
        setVoiceTranscript(update.data.voice_transcript || '')
        setCollection(update.data.collection)
        setDriveStopLatched(update.data.drive_stop_latched)
      } else if (update.type === 'health') {
        setHealth(update.data)
      } else if (update.type === 'voice') {
        setHealth(current => current ? { ...current, voice_state: update.data.state } : current)
        setVoiceTranscript(update.data.transcript || '')
      } else if (update.type === 'task') {
        setTask(current => preferNewestTask(current, update.data))
        setHistory(current => upsertTaskHistory(current, update.data))
      } else if (update.type === 'task_history') {
        setHistory(update.data.tasks)
      } else if (update.type === 'mapping') {
        setMapping(current => {
          if (update.data.kind === 'reset') return update.data.state
          if (!current) return current
          const next = { ...current }
          if (update.data.kind === 'map') next.map = update.data.map
          if (update.data.kind === 'pose') next.pose = update.data.pose
          if (update.data.kind === 'scan') next.scan = update.data.scan
          if (update.data.kind === 'path') next.path = update.data.path
          return next
        })
      } else if (update.type === 'perception') {
        setPerception(update.data)
      } else if (update.type === 'collection') {
        setCollection(update.data)
      } else if (update.type === 'drive_stop') {
        setDriveStopLatched(update.data.latched)
      }
    }
    events.onerror = () => setError(msg("状态流正在重连…"))
    events.onopen = () => setError('')
    return () => events.close()
  }, [])

  if (!bootstrap || !health) {
    return <main className="loading"><LanguageToggle /><p>{t("正在连接机器人…")}</p><small>{s(error)}</small></main>
  }

  const toggleBaseStop = async () => {
    setStopBusy(true)
    setError('')
    try {
      const response = driveStopLatched
        ? await api.operatorClearBaseStop()
        : await api.operatorStopBase()
      setDriveStopLatched(response.latched)
    } catch (reason) { setError(String(reason)) }
    finally { setStopBusy(false) }
  }

  return <div className="shell">
    <header className="app-header">
      <div className="brand-lockup">
        <span className="brand-mark">XL</span>
        <div><p className="eyebrow">XLEROBOT OPERATOR CONSOLE</p>
          <h1>{workspace === 'operator'
            ? operatorView === 'demo' ? t("Demo 演示") : t("手动控制")
            : t(workspaceNames[workspace]) || t("运行界面")}</h1></div>
      </div>
      <div className="header-status">
        <LanguageToggle />
        <div className="identity">
          <span>{t("版本")}<strong>{bootstrap.release}</strong></span>
          <span>{t("机器人")}<strong>{bootstrap.unit}</strong></span>
          <span>{t("场地")}<strong>{bootstrap.site || t("未激活")}</strong></span>
        </div>
        <div className={`readiness ${health.readiness.toLowerCase()}`}>
          <span />{health.readiness === 'READY' ? t("系统就绪")
            : health.readiness === 'DEGRADED' ? t("需要关注") : t("系统阻塞")}
        </div>
        {workspace === 'operator' && <button
          className={`header-base-stop ${driveStopLatched ? 'latched' : ''}`}
          onClick={toggleBaseStop} disabled={stopBusy}>
          <strong>{stopBusy ? t("正在处理…")
            : driveStopLatched ? t("解除底盘锁止") : t("锁止底盘")}</strong>
          <small>{t("底盘软件锁止 · 不是急停")}</small>
        </button>}
      </div>
    </header>
    {workspace === 'operator' && <nav className="operator-view-tabs"
      aria-label={t("Operator 视图")}>
      <button className={operatorView === 'demo' ? 'active' : ''}
        onClick={() => setOperatorView('demo')}>{t("Demo 演示")}</button>
      <button className={operatorView === 'manual' ? 'active' : ''}
        onClick={() => setOperatorView('manual')}>{t("手动控制")}</button>
    </nav>}
    {error && <div className="notice">{s(error)}</div>}
    {workspace === 'operator' && <OperatorWorkspace
      health={health} task={task} history={history} mapping={mapping}
      places={bootstrap.named_places || []} perception={perception}
      voiceTranscript={voiceTranscript}
      view={operatorView} driveStopLatched={driveStopLatched}
      onTask={incoming => setTask(current => preferNewestTask(current, incoming))}
      onError={setError} />}
    {workspace === 'mapping' && <MappingWorkspace state={mapping}
      phase={bootstrap.mapping_phase} initialSiteId={bootstrap.site} onError={setError} />}
    {workspace === 'calibration' && <CalibrationWorkspace
      unitId={bootstrap.unit} workflow={bootstrap.calibration_workflow}
      captureOnly={bootstrap.calibration_capture_only === true}
      onError={setError} />}
    {workspace === 'collection' && <CollectionWorkspace
      state={collection} initialDatasetId={bootstrap.default_dataset_id}
      storage={bootstrap.collection_storage} health={health}
      onState={setCollection} onError={setError} />}
  </div>
}

export function CollectionWorkspace({ state, initialDatasetId, storage, health, onState, onError }: {
  state: CollectionState | null
  initialDatasetId: string
  storage?: { root: string, config_file: string } | null
  health?: Health | null
  onState: (state: CollectionState | null) => void
  onError: (message: Message) => void
}) {
  const { t, s } = useLanguage()
  const [template, setTemplate] = useState<'pick' | 'manual'>('pick')
  const [datasetId, setDatasetId] = useState(
    initialDatasetId || 'xlerobot-glue-stick-grasp-30',
  )
  const [objectId, setObjectId] = useState('羽毛球')
  const [instruction, setInstruction] = useState('抓住羽毛球')
  const [duration, setDuration] = useState(30)
  const [homePending, setHomePending] = useState(false)
  const [startPending, setStartPending] = useState(false)
  const [recoveryPending, setRecoveryPending] = useState(false)
  const [recoveryMessage, setRecoveryMessage] = useState<Message>('')
  const [needsReset, setNeedsReset] = useState(false)
  const [reviewPending, setReviewPending] = useState(false)
  const [reviewMessage, setReviewMessage] = useState<Message>('')
  const [episodes, setEpisodes] = useState<CollectionEpisode[]>([])
  const [historyError, setHistoryError] = useState<Message>('')
  const [historyRevision, setHistoryRevision] = useState(0)
  const collectionStateRef = useRef(state)
  collectionStateRef.current = state
  useEffect(() => { setReviewMessage('') }, [state?.episode_id])
  useEffect(() => {
    let canceled = false
    setEpisodes([])
    setHistoryError('')
    api.collectionEpisodes(datasetId).then(result => {
      if (!canceled) setEpisodes(result.episodes)
    }).catch(reason => { if (!canceled) setHistoryError(msg("最近记录加载失败：{0}", String(reason))) })
    return () => { canceled = true }
  }, [datasetId, state?.episode_id, state?.status, historyRevision])
  const recover = async (operation: 'release' | 'reset') => {
    if (recoveryPending || startPending) return
    setRecoveryPending(true)
    try {
      const result = await api.recoverCollection(operation)
      onState(result.collection)
      setNeedsReset(operation === 'release')
      setRecoveryMessage(result.message)
      onError('')
    } catch (reason) { onError(String(reason)) }
    finally { setRecoveryPending(false) }
  }
  const running = state?.status === 'RUNNING'
  const poseWarning = health?.diagnostics['xlerobot/collection_initial_pose']
  const abortable = running && ![
    'STOPPING_RECORDING', 'FINALIZING', 'REVIEW',
  ].includes(state?.phase || '')
  const start = async (dryRun: boolean) => {
    if (startPending || recoveryPending || needsReset) return
    setRecoveryMessage('')
    setStartPending(true)
    const episodeId = `episode-${new Date().toISOString().replace(/\D/g, '').slice(0, 17)}`
    try {
      onState(await api.startCollection({
        template_id: template, dataset_id: datasetId, episode_id: episodeId,
        collection_profile_id: 'two_wheel_pick', object_id: objectId,
        language_instruction: instruction, max_duration_s: duration, dry_run: dryRun,
      }))
    } catch (reason) { onError(String(reason)) }
    finally { setStartPending(false) }
  }
  const finish = async () => {
    if (!state) return
    try {
      onState(await api.finalizeCollection(
        state.dataset_id, state.episode_id,
      ))
    }
    catch (reason) { onError(String(reason)) }
  }
  const abort = async () => {
    if (!state) return
    try {
      onState(await api.cancelCollection(
        state.dataset_id, state.episode_id,
      ))
    }
    catch (reason) { onError(String(reason)) }
  }
  const home = async () => {
    if (!state || homePending || recoveryPending) return
    setHomePending(true)
    try { await api.beginCollection(state.dataset_id, state.episode_id) }
    catch (reason) { onError(String(reason)) }
    finally { setHomePending(false) }
  }
  useEffect(() => {
    const keydown = (event: KeyboardEvent) => {
      const target = event.target instanceof Element ? event.target : null
      if (recoveryPending || event.repeat || event.isComposing || event.ctrlKey || event.altKey || event.metaKey || event.shiftKey ||
          target?.closest('input, textarea, select, [contenteditable]')) return
      if (event.key === 'Home' && running && state?.phase === 'WAITING_HOME') {
        event.preventDefault()
        void home()
      } else if (event.key === 'End' && running && state?.phase === 'RECORDING') {
        event.preventDefault()
        void finish()
      }
    }
    window.addEventListener('keydown', keydown)
    return () => window.removeEventListener('keydown', keydown)
  })
  const review = async (episode: { dataset_id: string, episode_id: string }, status: 'accepted' | 'rejected') => {
    if (reviewPending) return
    setReviewPending(true)
    try {
      await api.reviewEpisode(
        episode.dataset_id, episode.episode_id, status,
      )
      const current = collectionStateRef.current
      if (current && current.dataset_id === episode.dataset_id && current.episode_id === episode.episode_id) {
        onState({ ...current, review_status: status })
      }
      setHistoryRevision(value => value + 1)
      setReviewMessage(msg("{0}：{1}", episode.episode_id, status === 'accepted' ? msg("已保留，将用于后续转换。") : msg("已拒绝，后续转换将跳过，原始文件仍保留。")))
      onError('')
    } catch (reason) { setReviewMessage(msg("保留状态保存失败：{0}", String(reason))) }
    finally { setReviewPending(false) }
  }
  return <section className="engineering-card collection-card">
    <p className="section-label">FOLLOWER NEXT-STATE · NPZ + DUAL MP4</p>
    <h2>{t("ACT 数据采集")}</h2>
    {!running && poseWarning && poseWarning.level > 0 && <div className="collection-warning" role="alert">
      <h3>{poseWarning.level >= 2 ? t("初始姿态不合适，暂不能开始准备") : t("正在等待主从臂反馈")}</h3>
      <p>{s(poseWarning.message)}</p>
      <p>{t("不要强行掰动上力的关节。先托住主从臂 → 释放主从臂扭矩 → 将提示的关节摆回允许范围 → Reset → 开始。")}</p>
      <p>{t("若提示头部或左臂，请先停止运行并释放对应扭矩；“释放主从臂”按钮不会释放头部和左臂。")}</p>
    </div>}
    {state?.status === 'FAILED' && <div className="collection-warning" role="alert">
      <h3>{t("本次采集未能完成 · ")}{state.phase}</h3><p>{s(state.message)}</p>
      <p>{t("若涉及姿态或控制器：先托住主从臂，释放扭矩，调整姿态后 Reset，再点击开始。完整错误信息保留在这里。")}</p>
    </div>}
    <div className="workflow-tabs">
      <button disabled={running || startPending} className={template === 'pick' ? 'active' : ''} onClick={() => setTemplate('pick')}>{t("抓取模板")}</button>
      <button disabled={running || startPending} className={template === 'manual' ? 'active' : ''} onClick={() => setTemplate('manual')}>{t("通用手动")}</button>
    </div>
    <div className="camera-previews">
      <figure><img src="/api/v1/cameras/head/stream" alt={t("头部 D455 实时预览")} />
        <figcaption>HEAD · D455</figcaption></figure>
      <figure><img src="/api/v1/cameras/wrist/stream" alt={t("右腕相机实时预览")} />
        <figcaption>WRIST · RIGHT ARM</figcaption></figure>
    </div>
    <p className="hint">{t("数据集名称：多条采样共用一个名称。每次开始会自动生成独立的采样编号。")}</p>
    {storage && <div className="collection-storage">
      <strong>{t("保存在机器人主机，不是浏览器所在电脑")}</strong>
      <p>{t("当前数据集目录：")}<code>{storage.root}/datasets/{datasetId}</code></p>
      <p>{t("原始数据：")}<code>{t("raw/<采样编号>/")}</code>{t("；保留 / 拒绝记录：")}<code>reviews/</code></p>
      <p>{t("配置文件：")}<code>{storage.config_file || t("当前通过 ROS launch 启动，未指定配置文件")}</code></p>
      <p>{t("修改根目录：配置中的 ")}<code>data.collection_root</code>{t("；修改后重新启动数采。转换时将 ")}<code>data.dataset_root</code> {t(" 指向上面的完整数据集目录。修改位置不会搬迁旧数据。")}</p>
    </div>}
    <div className="field-row"><input aria-label={t("数据集名称")} value={datasetId} disabled={running || startPending || reviewPending} onChange={event => setDatasetId(event.target.value)} placeholder="Dataset ID" />
      <input value={objectId} disabled={running || startPending} onChange={event => {
        setObjectId(event.target.value)
        setInstruction(`抓住${event.target.value}`)
      }} placeholder={t("物体标签")} /></div>
    <label>{t("语言指令")}<input value={instruction} disabled={running || startPending} onChange={event => setInstruction(event.target.value)} /></label>
    <p className="hint">{t('界面语言不修改物体标签和训练指令；如需英文训练指令，请手动编辑。')}</p>
    <label>{t("最长时长 ")}{duration}s<input disabled={running || startPending} type="range" min="5" max="120" value={duration}
      onChange={event => setDuration(Number(event.target.value))} /></label>
    {!running && <div className="field-row"><button disabled={startPending || recoveryPending || needsReset || reviewPending} onClick={() => start(true)}>{t("状态机 dry-run")}</button>
      <button disabled={startPending || recoveryPending || needsReset || reviewPending} className="primary" onClick={() => start(false)}>{t("开始 / 准备 pregrasp")}</button></div>}
    <div className="field-row">
      <button className="danger" disabled={startPending || recoveryPending}
        onClick={() => recover('release')}>{t("释放主从臂扭矩")}</button>
      <button disabled={running || startPending || recoveryPending || reviewPending}
        onClick={() => recover('reset')}>{t("Reset / 重置状态")}</button>
    </div>
    <p className="hint">{t("异常恢复：先托住主从臂 → 释放扭矩（中止当前条，保留 incomplete）→ 手动摆好 → Reset → 开始。Reset 不上力、不回位、不删除数据；释放时底盘一并停用，头部保持。")}</p>
    {recoveryPending && <p role="status">{t("正在停止采集或处理恢复，请稍候…")}</p>}
    {recoveryMessage && <p role="status">{s(recoveryMessage)}</p>}
    {running && <div className="field-row">
      <button className="primary" onClick={home}
        disabled={state.phase !== 'WAITING_HOME' || homePending || recoveryPending}>
        {t("Home / 释放主臂并开始采集 ")}</button>
      <button className="primary" onClick={finish}
        disabled={state.phase !== 'RECORDING' || recoveryPending}>
        {t("End / 结束并保存到本机 ")}</button>
      <button className="danger" onClick={abort} disabled={!abortable || recoveryPending}>
        {t("Abort / 中止并保留 incomplete ")}</button>
    </div>}
    {state && <p>{t("本条采样：")}{state.episode_id}</p>}
    {state && <div className="collection-progress"><strong>{state.phase}</strong>
      <progress max="1" value={state.progress} /><span>{state.frame_count} frames · {state.elapsed_s.toFixed(1)}s · {s(state.message)}</span></div>}
    {state?.status === 'SUCCEEDED' && !state.dry_run && state.episode_uri &&
      <div className="field-row">
      <span>{t("本条：")}{state.review_status === 'accepted' ? t("已保留") : state.review_status === 'rejected' ? t("已拒绝") : t("未选择（不进入训练）")}</span>
      {state.review_status !== 'accepted' && <button disabled={reviewPending || recoveryPending} onClick={() => review(state, 'accepted')}>
        {state.review_status === 'rejected' ? t("恢复保留本条") : t("保留本条")}</button>}
      {state.review_status !== 'rejected' && <button disabled={reviewPending || recoveryPending} onClick={() => review(state, 'rejected')}>{t("拒绝本条")}</button>}</div>}
    {reviewPending && <p role="status">{t("正在保存保留状态…")}</p>}
    {reviewMessage && <p role="status">{t(reviewMessage)}</p>}
    <p className="hint">{t("新采样正常保存并通过完整性检查后默认保留，无需点击通过。只需拒绝不想用于训练的数据；拒绝可恢复，不删除文件。旧数据不自动改选。")}</p>
    <h3>{t("最近采样（当前数据集，最多 20 条）")}</h3>
    <button onClick={() => setHistoryRevision(value => value + 1)}>{t("刷新记录")}</button>
    {historyError && <p role="alert">{s(historyError)}</p>}
    {episodes.map(episode => <div className="field-row" key={episode.episode_id}>
      <span>{episode.episode_id} · {episode.frame_count} {t(" 帧 · ")}{episode.duration_s.toFixed(1)}s · {episode.review_status === 'accepted' ? t("已保留") : episode.review_status === 'rejected' ? t("已拒绝") : t("未选择（旧数据）")}</span>
      <button disabled={reviewPending || recoveryPending} aria-label={t("{0} {1}", episode.episode_id, episode.review_status === 'rejected' ? t("恢复保留") : t("拒绝"))}
        onClick={() => review(episode, episode.review_status === 'rejected' ? 'accepted' : 'rejected')}>
        {episode.review_status === 'rejected' ? t("恢复保留") : t("拒绝")}</button>
      {!episode.review_status && <button disabled={reviewPending || recoveryPending} aria-label={t("{0} 保留", episode.episode_id)}
        onClick={() => review(episode, 'accepted')}>{t("保留旧数据")}</button>}
    </div>)}
    <p className="hint">{t("End 只结束录制，遥操继续，可放下物品并手动归位；这些动作不进入已保存数据。下一次开始会结束当前遥操并准备新的 pregrasp。释放扭矩按钮会停止遥操。")}</p>
    <p className="hint">{t("开始：主从臂准备到 pregrasp 后保持上力（通用手动仅主臂对齐从臂）。等待 Home：托住主臂后点击或按键盘 Home；数据就绪后交接到遥操，显示 RECORDING 再示教。End 结束并保存到本机，不会上传。输入框内不响应快捷键。等待超过 60 秒将中止本条。")}</p>
  </section>
}

function CalibrationWorkspace({ unitId, workflow, captureOnly, onError }: {
  unitId: string, workflow: string, captureOnly: boolean,
  onError: (message: Message) => void
}) {
  const { t } = useLanguage()
  const [sourceUri, setSourceUri] = useState('')
  const [result, setResult] = useState<Message>('')
  const [nominalRadius, setNominalRadius] = useState('0.0635')
  const [nominalSeparation, setNominalSeparation] = useState('0.52')
  const [straightCommanded, setStraightCommanded] = useState('1.0,1.0')
  const [straightActual, setStraightActual] = useState('1.0,1.0')
  const [rotationCommanded, setRotationCommanded] = useState('6.283185,6.283185')
  const [rotationActual, setRotationActual] = useState('6.283185,6.283185')
  const [coverage, setCoverage] = useState<CalibrationCoverage | null>(null)
  const workflows: Record<string, string> = {
    servo: t("舵机标定"), base_geometry: t("底盘几何"),
    head_camera: t("头部 D455 外参"), right_handeye: t("右臂手眼"),
  }
  const preflight = async () => {
    try {
      const value = await api.calibrationPreflight(unitId, workflow)
      setResult(msg("{0} · {1}", msg(value.message), value.artifact_uri))
    } catch (reason) { onError(String(reason)) }
  }
  const importResult = async () => {
    try {
      const value = await api.importCalibration(unitId, workflow, sourceUri)
      setResult(msg("{0} · {1}", value.quality_passed ? msg("质量通过") : msg("质量未通过"), JSON.stringify(value.metrics)))
    } catch (reason) { onError(String(reason)) }
  }
  const activate = async () => {
    try {
      const value = await api.activateCalibration(unitId)
      setResult(msg("已原子激活，可回滚：{0}", value.artifact_uri))
    } catch (reason) { onError(String(reason)) }
  }
  const values = (text: string) => text.split(',').map(value => Number(value.trim()))
  const saveBaseGeometry = async () => {
    try {
      const value = await api.saveBaseGeometry({
        unit_id: unitId,
        nominal_wheel_radius_m: Number(nominalRadius),
        nominal_wheel_separation_m: Number(nominalSeparation),
        straight_commanded_m: values(straightCommanded),
        straight_actual_m: values(straightActual),
        rotation_commanded_rad: values(rotationCommanded),
        rotation_actual_rad: values(rotationActual),
      })
      setResult(msg("{0} · {1}", value.quality_passed ? msg("质量通过") : msg("质量未通过"), JSON.stringify(value.metrics)))
    } catch (reason) { onError(String(reason)) }
  }
  const visual = workflow === 'right_handeye'
  useEffect(() => {
    if (!visual) {
      setCoverage(null)
      return
    }
    let active = true
    let timer: ReturnType<typeof setTimeout>
    const poll = async () => {
      try {
        const value = await api.calibrationSampleCoverage()
        if (active) setCoverage(value)
      } catch {
        if (active) setCoverage(null)
      }
      if (active) timer = setTimeout(poll, 2000)
    }
    void poll()
    return () => { active = false; clearTimeout(timer) }
  }, [workflow, visual])
  if (workflow === 'servo') return <ServoCalibrationWorkspace
    unitId={unitId} captureOnly={captureOnly} onError={onError} />
  if (workflow === 'head_camera' || workflow === 'right_handeye') return <>
    <HeadCalibrationWorkspace unitId={unitId} handeye={workflow === 'right_handeye'} onError={onError} />
    {visual && coverage && <HandeyeCoveragePanel coverage={coverage} />}
  </>
  return <section className="engineering-card calibration-card">
    <p className="section-label">UNIT CALIBRATION · {unitId}</p>
    <h2>{workflows[workflow] || t("标定工具")}</h2>
    {captureOnly
      ? <p>{t("此页面只采集原始标定结果；请回到命令行用 tools/calibrate 严格求解、导入并激活。")}</p>
      : <><p>{t("所有结果先进入 draft；四项质量门槛全部通过后才能激活。激活不会修改仓库配置。")}</p>
        <button onClick={preflight}>{t("检查当前工作流")}</button></>}
    {workflow === 'base_geometry' ? <>
      <div className="field-row">
        <label>{t("当前轮径 m")}<input value={nominalRadius} onChange={event => setNominalRadius(event.target.value)} /></label>
        <label>{t("当前轮距 m")}<input value={nominalSeparation} onChange={event => setNominalSeparation(event.target.value)} /></label>
      </div>
      <label>{t("两次直行命令距离 m（逗号分隔）")}<input value={straightCommanded} onChange={event => setStraightCommanded(event.target.value)} /></label>
      <label>{t("两次直行实测距离 m")}<input value={straightActual} onChange={event => setStraightActual(event.target.value)} /></label>
      <label>{t("两次旋转命令角度 rad")}<input value={rotationCommanded} onChange={event => setRotationCommanded(event.target.value)} /></label>
      <label>{t("两次旋转实测角度 rad")}<input value={rotationActual} onChange={event => setRotationActual(event.target.value)} /></label>
      <button className="primary" onClick={saveBaseGeometry}>{captureOnly
        ? t("保存底盘测量结果") : t("计算、验收并保存 draft")}</button>
      <p className="hint">{t("至少两次直行和两次旋转；运动试验由当前独立 profile 执行，填写现场实测值后拟合。")}</p>
    </> : !captureOnly ? <>
      <label>{t("本地结果 URI")}<input value={sourceUri}
        onChange={event => setSourceUri(event.target.value)}
        placeholder=".xlerobot/staging/result.yaml" /></label>
      <button className="primary" onClick={importResult} disabled={!sourceUri}>{t("导入并验收")}</button>
    </> : null}
    {!captureOnly && <button onClick={activate}>{t("激活完整 calibration bundle")}</button>}
    {result && <p className="saved">{t(result)}</p>}
  </section>
}

export function HandeyeCoveragePanel({ coverage }: {
  coverage: CalibrationCoverage
}) {
  const { t } = useLanguage()
  const width = 360
  const height = 230
  const padding = 30
  const plotWidth = width - 2 * padding
  const plotHeight = height - 2 * padding
  const points = coverage.points
  const xCenter = points.length
    ? (Math.min(...points.map(point => point.x_m))
      + Math.max(...points.map(point => point.x_m))) / 2
    : 0
  const yCenter = points.length
    ? (Math.min(...points.map(point => point.y_m))
      + Math.max(...points.map(point => point.y_m))) / 2
    : 0
  const positionSpan = Math.max(
    coverage.spans_m.x,
    coverage.spans_m.y,
    0.001,
  )
  const scale = Math.min(plotWidth, plotHeight) / positionSpan
  const minimumZ = points.length
    ? Math.min(...points.map(point => point.z_m))
    : 0
  const zSpan = Math.max(coverage.spans_m.z, 0.001)
  const millimeters = (value: number) => `${(value * 1000).toFixed(1)} mm`
  return <section className="handeye-coverage">
    <div className="coverage-heading">
      <div><strong>{t("姿态覆盖事实")}</strong><small>{t("XY 俯视 · 点色表示 Z 高度")}</small></div>
      <span>{coverage.sample_count} samples</span>
    </div>
    <div className="coverage-grid">
      <svg viewBox={`0 0 ${width} ${height}`}
        aria-label={t("右手眼 XY 姿态覆盖")} role="img">
        <rect x={padding} y={padding} width={plotWidth} height={plotHeight}
          className="coverage-frame" />
        <line x1={padding} y1={height - padding}
          x2={width - padding} y2={height - padding} />
        <line x1={padding} y1={padding}
          x2={padding} y2={height - padding} />
        <text x={width - padding} y={height - 9}>X</text>
        <text x={9} y={padding}>Y</text>
        {points.map(point => {
          const zFraction = Math.min(
            1, Math.max(0, (point.z_m - minimumZ) / zSpan),
          )
          return <circle key={point.index} data-sample-index={point.index}
            cx={width / 2 + (point.x_m - xCenter) * scale}
            cy={height / 2 - (point.y_m - yCenter) * scale}
            r="6" fill={`hsl(${205 - zFraction * 155} 78% 61%)`}>
            <title>{`#${point.index + 1} · X ${millimeters(point.x_m)} · Y ${millimeters(point.y_m)} · Z ${millimeters(point.z_m)}`}</title>
          </circle>
        })}
        {!points.length &&
          <text className="coverage-empty" x={width / 2} y={height / 2}>
            {t("暂无已恢复样本 ")}</text>}
      </svg>
      <dl className="coverage-metrics">
        <div><dt>X span</dt><dd>{millimeters(coverage.spans_m.x)}</dd></div>
        <div><dt>Y span</dt><dd>{millimeters(coverage.spans_m.y)}</dd></div>
        <div><dt>Z span</dt><dd>{millimeters(coverage.spans_m.z)}</dd></div>
        <div><dt>{t("最大两两姿态角")}</dt>
          <dd>{coverage.max_pairwise_pose_angle_deg.toFixed(1)}°</dd></div>
      </dl>
    </div>
    <p>{t("仅显示已保存样本的几何覆盖，不据此增加或推断质量通过阈值。")}</p>
  </section>
}

export function MappingWorkspace({ state, phase, initialSiteId, onError }: {
  state: MappingState | null, phase: string, initialSiteId: string,
  onError: (message: Message) => void
}) {
  const { t, s } = useLanguage()
  const [linearSpeed, setLinearSpeed] = useState(0.08)
  const [angularSpeed, setAngularSpeed] = useState(0.35)
  const siteId = initialSiteId || 'home'
  const [mapName, setMapName] = useState('ground-floor')
  const [placeId, setPlaceId] = useState('table')
  const [saved, setSaved] = useState<Message>('')
  const [site, setSite] = useState<SiteSummary | null>(null)
  const [siteError, setSiteError] = useState<Message>('')
  const [pending, setPending] = useState<Message>('')
  const busy = useRef(false)
  const [localized, setLocalized] = useState(false)
  const { armed, setArmed, connected, move: drive, stop } = useBaseTeleop(
    phase !== 'build' || !!pending, onError,
  )
  const refreshSite = async () => {
    const result = await api.site(siteId)
    setSite(result)
    setSiteError('')
  }
  useEffect(() => {
    let disposed = false
    const refresh = async () => {
      try {
        const result = await api.site(siteId)
        if (!disposed) { setSite(result); setSiteError('') }
      } catch (reason) { if (!disposed) setSiteError(msg("无法读取保存状态：{0}", String(reason))) }
    }
    void refresh()
    const timer = window.setInterval(refresh, 5000)
    return () => { disposed = true; window.clearInterval(timer) }
  }, [siteId])
  const perform = async (label: Message, action: () => Promise<void>) => {
    if (busy.current) return
    busy.current = true
    stop()
    setArmed(false)
    setPending(label)
    setSaved('')
    try {
      await action()
      try { await refreshSite() }
      catch (reason) { setSiteError(msg("操作已完成，但刷新保存状态失败：{0}", String(reason))) }
    } catch (reason) { onError(String(reason)) }
    finally { busy.current = false; setPending('') }
  }
  const save = async () => {
    await perform(msg("保存地图中…"), async () => {
      await api.saveMap(siteId, mapName.trim())
      setSaved(msg("地图已保存为草稿，尚未替换 Demo 地图。继续建图后请再次保存。"))
    })
  }
  const resetMap = async () => {
    if (!window.confirm(t("清除当前正在构建的地图并重新建图？未保存的建图结果无法恢复。")
      + t("已保存的地图、地点和 Demo 配置不会删除；重建后请重新确认或记录地点。"))) return
    await perform(msg("清除当前地图中…"), async () => {
      await api.resetMap()
      setSaved(msg("当前地图已清除，等待新扫描重新生成。已保存的地图和地点保持不变。"))
    })
  }
  const savePlace = async () => {
    const id = placeId.trim()
    if (site?.draft.places.some(place => place.id === id)
      && !window.confirm(t("用机器人当前的位置和朝向覆盖 {0}？", id))) return
    await perform(msg("记录地点中…"), async () => {
      await api.setPlace(siteId, id, id === 'table', id === 'table' ? 0.25 : 0)
      setSaved(msg("已记录 {0} 的位置和朝向。", id))
    })
  }
  const removePlace = async () => {
    if (!window.confirm(t("删除地点 {0}？", placeId.trim()))) return
    await perform(msg("删除地点中…"), async () => {
      await api.removePlace(siteId, placeId.trim())
      setSaved(msg("已删除 {0}。", placeId.trim()))
    })
  }
  const validateLocalization = async () => {
    setLocalized(false)
    await perform(msg("自动定位中，机器人会转动…"), async () => {
      const result = await api.validateLocalization(siteId)
      setLocalized(true)
      setSaved(msg("定位通过：σxy {0} m，σyaw {1} rad", result.position_stddev_m.toFixed(3), result.yaw_stddev_rad.toFixed(3)))
    })
  }
  const validatePlace = async () => {
    await perform(msg("导航验证中，机器人会移动…"), async () => {
      const result = await api.validatePlace(siteId, placeId)
      setSaved(msg("{0} 通过：位置误差 {1} m，角度误差 {2} rad{3}", placeId, result.position_error_m.toFixed(3), result.yaw_error_rad.toFixed(3), result.site_ready ? msg("；site 已可激活") : ''))
    })
  }
  const activate = async () => {
    await perform(msg("激活场地中…"), async () => {
      const result = await api.activateSite(siteId)
      setSaved(msg("场地已激活：{0}。下次 Demo 使用的 site.map / site.places 仍由本机配置指定，请指向该场地 current/ 中的文件。", result.artifact_uri))
    })
  }
  const validId = (value: string) => /^[A-Za-z0-9][A-Za-z0-9_.-]{0,95}$/.test(value.trim())
  const places = site?.draft?.places || []
  const selectedPlace = places.find(place => place.id === placeId.trim())

  return <section className="mapping-grid">
    <article className="map-card">
      <div className="map-heading"><div><p className="section-label">LIVE SLAM</p>
        <h2>{t("场地地图")}</h2></div><strong>{state?.slam || 'WAITING'}</strong></div>
      <MapCanvas state={state} places={places} />
      {state?.reset_notice && <p className="hint">{s(state.reset_notice)}</p>}
      <div className="map-stats">
        <span>{t("分辨率 ")}<b>{state?.map?.resolution.toFixed(3) || '—'} m</b></span>
        <span>{t("尺寸 ")}<b>{state?.map ? `${state.map.width} × ${state.map.height}` : '—'}</b></span>
        <span>Pose <b>{state?.pose ? `${state.pose.x.toFixed(2)}, ${state.pose.y.toFixed(2)}` : '—'}</b></span>
      </div>
    </article>
    <div className="mapping-side">
      {phase === 'build' && <article className="drive-card">
        <div className="drive-title"><div><p className="section-label">DEAD-MAN TELEOP</p><h2>{t("覆盖场地")}</h2></div>
          <button disabled={!!pending} className={armed ? 'armed' : ''} onClick={() => { stop(); setArmed(!armed) }}>
            {armed ? t("结束遥控") : t("开启遥控")}
          </button></div>
        <JoystickPad disabled={!armed || !!pending}
          onMove={(linear, angular) => drive(linear * linearSpeed, angular * angularSpeed)}
          onRelease={() => drive(0, 0)} />
        <p className="hint">{!armed ? t("遥控已关闭") : connected ? t("已连接 · 松手零速保持") : t("已开启 · 拖动摇杆连接")}</p>
        <button className="block stop-drive" onClick={() => { stop(); setArmed(false) }}>{t("停车并结束遥控")}</button>
        <label>{t("最大线速度 ")}{linearSpeed.toFixed(2)} m/s<input type="range" min="0.03" max="0.12" step="0.01"
          value={linearSpeed} onChange={event => setLinearSpeed(Number(event.target.value))} /></label>
        <label>{t("最大角速度 ")}{angularSpeed.toFixed(2)} rad/s<input type="range" min="0.15" max="0.50" step="0.05"
          value={angularSpeed} onChange={event => setAngularSpeed(Number(event.target.value))} /></label>
      </article>}
      <article className="site-card">
        <p className="section-label">SITE · {phase.toUpperCase()}</p>
        <h2>{phase === 'validate' ? t("验证并激活") : t("保存地图与地点")}</h2>
        <p className="hint">{t("场地：")}{siteId} · {!site ? t("正在读取保存状态…")
          : site.active_version ? t("已激活版本 {0}", site.active_version) : t("本次场地未激活")}</p>
        {site && <p className="hint">{site.draft?.map_saved
          ? t("已保存地图：{0} · {1}", site.draft.map_name, new Date(site.draft.map_saved_at).toLocaleString())
          : t("尚无地图草稿")} {t(" · 已记录 ")}{places.length} {t(" 个地点")}</p>}
        {siteError && <p role="alert">{s(siteError)}</p>}
        {phase === 'build' && <>
          <label className="site-field">{t("地图名")}<input aria-label={t("地图名")} disabled={!!pending} value={mapName}
            onChange={event => setMapName(event.target.value)} /></label>
          <button className="primary block" disabled={armed || !!pending || !state?.map || !validId(mapName)} onClick={save}>{t("保存当前地图")}</button>
          <button className="danger block" disabled={armed || !!pending} onClick={resetMap}>{t("清除当前地图并重建")}</button>
          <p className="hint">{t("先停车并结束遥控，等机器人停稳再保存。保存不会结束建图；结束前再保存一次。")}</p>
        </>}
        <label className="site-field">{t("地点 ID")}<input aria-label={t("地点 ID")} disabled={!!pending} value={placeId}
          onChange={event => setPlaceId(event.target.value)} placeholder="table / home" /></label>
        <p className="hint">{placeId.trim() === 'table'
          ? t("table：记录最终桌边停车位和面向桌面的朝向；导航先在后方 0.25 m 停靠，再精确贴桌。")
          : t("记录机器人当前位置和朝向，不进行贴桌。")} {t(" ID 使用英文字母、数字、下划线或连字符。")}</p>
        {phase === 'build' && <div className="validation-actions">
          <button disabled={armed || !!pending || !state?.pose || !validId(placeId)} onClick={savePlace}>
            {selectedPlace ? t("用当前位置更新地点") : t("记录当前位置为地点")}</button>
          <button className="quiet block" disabled={armed || !!pending || !selectedPlace} onClick={removePlace}>{t("删除所选地点")}</button>
        </div>}
        <ul className="saved-places" aria-label={t("已保存地点")}>
          {places.map(place => <li key={place.id}><button disabled={!!pending}
            aria-pressed={place.id === placeId.trim()} onClick={() => setPlaceId(place.id)}>
            <strong>{place.id}</strong> {place.x.toFixed(2)}, {place.y.toFixed(2)} m · {(place.yaw * 180 / Math.PI).toFixed(0)}°
            <small>{place.dock ? t("精确贴桌") : t("普通导航")} · {place.validated ? t("验证通过") : t("待验证")}</small>
          </button></li>)}
        </ul>
        {phase === 'validate' && <div className="validation-actions">
          <button disabled={!!pending || !site?.draft?.map_saved} onClick={validateLocalization}>{t("1. 自动定位验证")}</button>
          <button disabled={!!pending || !localized || !selectedPlace} onClick={validatePlace}>{t("2. 导航并验证地点")}</button>
          <button disabled={!!pending || !site?.draft?.ready} className="primary block" onClick={activate}>{t("3. 激活场地草稿")}</button>
        </div>}
        <p role="status" className="saved">{t(pending || saved)}</p>
        <p className="hint">{phase === 'validate'
          ? t("先完成 AutoLocalize，再逐地点导航；table 会执行 position-only、Spin 和精确 dock。所有地点通过后才允许激活。")
          : t("顺序：遥控覆盖场地 → 停稳并记录地点 → 最后保存地图 → 停止 build，使用同一配置启动 validate。草稿不会自动替换正在使用的 Demo 地图。")}</p>
      </article>
    </div>
  </section>
}

function MapCanvas({ state, places = [], showPath = false, personTarget = null }: {
  state: MappingState | null, places?: NamedPlace[], showPath?: boolean,
  personTarget?: PerceptionState['target'] | null,
}) {
  const { t } = useLanguage()
  const canvas = useRef<HTMLCanvasElement>(null)
  useEffect(() => {
    const element = canvas.current
    const map = state?.map
    if (!element || !map) return
    element.width = map.width
    element.height = map.height
    const context = element.getContext('2d')!
    const image = context.createImageData(map.width, map.height)
    map.data.forEach((value, index) => {
      const shade = value < 0 ? 24 : value === 0 ? 211 : Math.max(15, 211 - value * 1.8)
      const target = ((map.height - 1 - Math.floor(index / map.width)) * map.width + index % map.width) * 4
      image.data[target] = shade * .82
      image.data[target + 1] = shade * .95
      image.data[target + 2] = shade
      image.data[target + 3] = 255
    })
    context.putImageData(image, 0, 0)
    const worldToPixel = (x: number, y: number, yaw: number) => {
      const dx = x - map.origin.x
      const dy = y - map.origin.y
      const cos = Math.cos(-map.origin.yaw)
      const sin = Math.sin(-map.origin.yaw)
      return {
        x: (cos * dx - sin * dy) / map.resolution,
        y: map.height - 1 - (sin * dx + cos * dy) / map.resolution,
        yaw: yaw - map.origin.yaw,
      }
    }
    const drawRobotPose = (
      x: number, y: number, yaw: number, stroke: string, fill: string,
    ) => {
      const pose = worldToPixel(x, y, yaw)
      const robotMinX = -0.19565 / map.resolution
      const robotMaxX = 0.205 / map.resolution
      const robotHalfWidth = 0.265 / map.resolution
      const robotLength = robotMaxX - robotMinX
      const robotWidth = robotHalfWidth * 2
      context.save()
      context.translate(pose.x, pose.y)
      context.rotate(-pose.yaw)
      context.fillStyle = fill
      context.strokeStyle = stroke
      context.lineWidth = 1.5
      context.fillRect(robotMinX, -robotHalfWidth, robotLength, robotWidth)
      context.strokeRect(robotMinX, -robotHalfWidth, robotLength, robotWidth)
      context.strokeStyle = '#f8fafc'
      context.fillStyle = '#f8fafc'
      context.lineWidth = Math.max(1.5, robotWidth * .08)
      context.lineCap = 'round'
      context.beginPath()
      context.moveTo(robotMinX + robotLength * .22, 0)
      context.lineTo(robotMaxX - robotLength * .18, 0)
      context.stroke()
      context.beginPath()
      context.moveTo(robotMaxX - robotLength * .12, 0)
      context.lineTo(robotMaxX - robotLength * .30, -robotWidth * .22)
      context.lineTo(robotMaxX - robotLength * .30, robotWidth * .22)
      context.closePath()
      context.fill()
      context.restore()
      return pose
    }
    context.font = `${Math.max(8, Math.min(map.width, map.height) * .032)}px sans-serif`
    if (showPath && state?.path && state.path.points_xy.length > 1) {
      context.strokeStyle = '#5de4a6'
      context.lineWidth = Math.max(1.5, 0.035 / map.resolution)
      context.lineJoin = 'round'
      context.lineCap = 'round'
      context.beginPath()
      state.path.points_xy.forEach(([x, y], index) => {
        const pixel = worldToPixel(x, y, 0)
        if (index === 0) context.moveTo(pixel.x, pixel.y)
        else context.lineTo(pixel.x, pixel.y)
      })
      context.stroke()
    }
    places.forEach((place, index) => {
      const color = placePalette[index % placePalette.length]
      drawRobotPose(
        place.x, place.y, place.yaw, color.stroke, color.fill,
      )
    })
    if (!state?.pose) return
    if (personTarget?.frame_id === 'map') {
      const person = worldToPixel(personTarget.x, personTarget.y, 0)
      const radius = Math.max(4, 0.12 / map.resolution)
      context.fillStyle = 'rgba(255, 114, 128, .24)'
      context.strokeStyle = '#ff7280'
      context.lineWidth = 2
      context.beginPath()
      context.arc(person.x, person.y, radius, 0, Math.PI * 2)
      context.fill()
      context.stroke()
      context.fillStyle = '#ffd6da'
      context.fillText(t("人员"), person.x + radius + 4, person.y + 4)
    }
    if (state.scan) {
      context.fillStyle = 'rgba(84, 237, 173, .7)'
      state.scan.points_xy.forEach(([baseX, baseY]) => {
        const mapX = state.pose!.x
          + Math.cos(state.pose!.yaw) * baseX - Math.sin(state.pose!.yaw) * baseY
        const mapY = state.pose!.y
          + Math.sin(state.pose!.yaw) * baseX + Math.cos(state.pose!.yaw) * baseY
        const pixel = worldToPixel(mapX, mapY, 0)
        const px = pixel.x
        const py = pixel.y
        context.fillRect(px, py, 1.5, 1.5)
      })
    }
    drawRobotPose(
      state.pose.x, state.pose.y, state.pose.yaw,
      '#63aef7', 'rgba(99, 174, 247, .34)',
    )
  }, [state, places, showPath, personTarget, t])
  if (!state?.map) return <div className="map-empty">
    <span className="map-loader" />
    <strong>{t("正在读取导航地图")}</strong>
    <small>{t("等待 /map 的已激活场地资产")}</small>
  </div>
  return <div className="map-stage">
    <canvas ref={canvas} className="map-canvas" />
    <div className="map-legend"><span className="robot-frame" />{t("机器人当前位姿 ")}{places.map((place, index) => {
        const color = placePalette[index % placePalette.length]
        return <span className="legend-place" key={place.id}>
          <i style={{ borderColor: color.stroke, background: color.fill }} />
          {t(placeNames[place.id]) || place.id}
        </span>
      })}
      {showPath && <><span className="path-line" />{t("当前路径")}</>}
      {personTarget && <><span className="person-point" />{t("人员目标")}</>}</div>
  </div>
}

function OperatorWorkspace({
  health, task, history, mapping, places, perception, voiceTranscript,
  view, driveStopLatched, onTask, onError,
}: {
  health: Health
  task: Task | null
  history: Task[]
  mapping: MappingState | null
  places: NamedPlace[]
  perception: PerceptionState | null
  voiceTranscript: string
  view: 'demo' | 'manual'
  driveStopLatched: boolean | null
  onTask: (task: Task | null) => void
  onError: (message: Message) => void
}) {
  const { t } = useLanguage()
  const [objectId, setObjectId] = useState('')
  const [graspBackend, setGraspBackend] = useState<'act' | 'centroid' | 'gpd'>('act')
  const [placeId, setPlaceId] = useState(
    places.find(place => place.id === 'table')?.id || places[0]?.id || '',
  )
  const [manualBusy, setManualBusy] = useState(false)
  const [manualOperation, setManualOperation] = useState('')
  const [manualResult, setManualResult] = useState<Message>('')
  const [personBoxVisible, setPersonBoxVisible] = useState(false)
  const running = task && !['SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED'].includes(task.status)
  const demoReady = health.execute_task_available
    && health.requirements.execute_task_live_ready === true
    && driveStopLatched === false
  const cameraPerception = activeCameraPerception(task, perception)
  const personTarget = activePersonTarget(task, perception)
  const shownPerception = cameraPerception?.kind === 'person' && !personBoxVisible
    ? null : cameraPerception
  const showNavigationPath = taskShowsNavigationPath(task)
    || manualOperation === 'navigate'
  const operationLabel: Record<string, string> = {
    localize: t("正在重新定位"), navigate: t("正在导航"),
    arm_ready: t("右臂正在回 Ready"), head_ready: t("头部正在回正"),
    gripper_open: t("正在打开右夹爪"),
  }
  useEffect(() => {
    if (cameraPerception?.kind !== 'person') {
      setPersonBoxVisible(false)
      return
    }
    setPersonBoxVisible(true)
    const timer = window.setTimeout(() => setPersonBoxVisible(false), 1500)
    return () => window.clearTimeout(timer)
  }, [cameraPerception?.observation_id, cameraPerception?.kind])
  useEffect(() => {
    if (!places.some(place => place.id === placeId)) {
      setPlaceId(places.find(place => place.id === 'table')?.id || places[0]?.id || '')
    }
  }, [places, placeId])
  const submit = async (event: FormEvent) => {
    event.preventDefault()
    onError('')
    try {
      onTask(await api.submitTask(objectId.trim(), graspBackend))
      setObjectId('')
    } catch (reason) { onError(String(reason)) }
  }
  const cancel = async () => {
    if (!task) return
    try { onTask(await api.cancelTask(task.task_id)) }
    catch (reason) { onError(String(reason)) }
  }
  const manual = async (operation: 'localize' | 'navigate' |
    'arm_ready' | 'head_ready' | 'gripper_open') => {
    setManualBusy(true)
    setManualOperation(operation)
    try {
      if (operation === 'localize') {
        const value = await api.operatorLocalize()
        setManualResult(msg("定位完成：σxy {0} m", value.position_stddev_m.toFixed(3)))
      } else if (operation === 'navigate') {
        const value = await api.operatorNavigate(placeId)
        setManualResult(msg("已到达 {0}", value.place_id))
      } else {
        const value = await api.operatorPreset(operation)
        setManualResult(msg("Preset 完成：{0}", value.preset))
      }
    } catch (reason) { onError(String(reason)) }
    finally { setManualBusy(false); setManualOperation('') }
  }
  return <div className="operator-workspace">
    <section className="status-strip" aria-label={t("关键运行状态")}>
      <StatusMetric label={t("主 Demo")} value={demoReady ? t("就绪")
        : health.execute_task_available ? t("等待依赖") : t("服务不可用")}
        state={demoReady ? 'ok' : 'bad'} />
      <StatusMetric label={t("语音")} value={t(voiceNames[health.voice_state]) || health.voice_state}
        state={health.voice_state === 'DISABLED' ? 'muted' : 'ok'} />
      <StatusMetric label={t("导航地图")} value={mapping?.map ? t("已加载") : t("等待地图")}
        state={mapping?.map ? 'ok' : 'bad'} />
      <StatusMetric label={t("机器人位置")} value={mapping?.pose ? t("正在跟踪") : t("等待定位")}
        state={mapping?.pose ? 'ok' : 'warn'} />
      <StatusMetric label={t("双路相机")} value={health.requirements.head_camera && health.requirements.wrist_camera
        ? t("画面正常") : t("等待画面")}
        state={health.requirements.head_camera && health.requirements.wrist_camera ? 'ok' : 'warn'} />
    </section>
    {view === 'demo' && <section className="hero-grid">
      <article className="task-card">
        <div className="card-heading"><div><p className="section-label">FETCH · DELIVER</p>
          <h2>{running ? t("正在取 {0}", task.object_id) : t("发起取物递送")}</h2></div>
          <span className="context-chip">{t("来源 · 桌边")}</span></div>
        {!running && <p className="card-intro">{t("输入物体名称，机器人将自动定位、前往桌边、抓取并递送给最近的人。")}</p>}
        {!running && <form onSubmit={submit}>
          <input value={objectId} onChange={event => setObjectId(event.target.value)}
            placeholder={t("输入物体名称，例如：羽毛球")} aria-label={t("物体名")} required />
          <select value={graspBackend} aria-label={t("抓取路线")}
            onChange={event => setGraspBackend(
              event.target.value as 'act' | 'centroid' | 'gpd',
            )}>
            <option value="act">{t("ACT 学习策略")}</option>
            <option value="centroid">{t("传统 · 质心顶抓")}</option>
            <option value="gpd">{t("传统 · GPD 顶抓")}</option>
          </select>
          <button className="primary" disabled={!demoReady}>
            {t("开始任务 ")}</button>
        </form>}
        {task && <TaskProgress task={task} />}
        {running && task.cancelable !== false
          && <button className="stop" onClick={cancel}>{t("停止当前任务")}</button>}
      </article>
      <article className="voice-card">
        <div className="card-heading"><div><p className="section-label">VOICE</p>
          <h2>{t("语音入口")}</h2></div><span className="live-badge">LIVE</span></div>
        <div className="voice-summary"><div className="voice-orb"><span /></div>
          <div><h3>{t(voiceNames[health.voice_state]) || health.voice_state}</h3>
            <p>{voiceTranscript ? t("识别结果：{0}", voiceTranscript)
              : t("说“小乐小乐”发起任务；状态会实时更新。")}</p></div></div>
      </article>
    </section>}
    <section className="operator-observation">
      <article className="map-card">
        <div className="map-heading"><div><p className="section-label">LIVE NAVIGATION</p>
          <h2>{t("地图与机器人位置")}</h2></div>
          <strong>{mapping?.pose ? t("正在跟踪") : t("等待定位")}</strong></div>
        <MapCanvas state={mapping} places={places}
          showPath={showNavigationPath}
          personTarget={personTarget} />
        <div className="map-stats compact">
          <span>{t("位置 ")}<b>{mapping?.pose
            ? `${mapping.pose.x.toFixed(2)}, ${mapping.pose.y.toFixed(2)} m` : '—'}</b></span>
          <span>{t("方向 ")}<b>{mapping?.pose ? `${(mapping.pose.yaw * 180 / Math.PI).toFixed(0)}°` : '—'}</b></span>
          <span>{t("预设地点 ")}<b>{places.length}</b></span>
        </div>
      </article>
      <article className="camera-card">
        <div className="card-heading"><div><p className="section-label">LIVE CAMERAS</p>
          <h2>{t("机器人视角")}</h2></div><span className="live-badge">LIVE</span></div>
        <div className="camera-previews">
          <CameraPreview camera="head" perception={shownPerception} />
          <CameraPreview camera="wrist" perception={null} />
        </div>
      </article>
    </section>
    {view === 'manual' && <section className="operator-controls">
      <article className="operator-console">
        <div className="console-heading">
          <div><p className="section-label">OPERATOR CONTROL</p>
            <h2>{t("观测与手动操作")}</h2>
            <p>{t("结合上方地图与相机画面进行定位、导航和姿态调整。任务执行时，手动命令自动锁定。")}</p>
          </div>
          <div className="console-status-actions">
            <span className={`console-mode ${running || driveStopLatched !== false ? 'busy' : 'idle'}`}>
              <i />{manualBusy ? operationLabel[manualOperation]
                : driveStopLatched ? t("底盘已软件锁止")
                  : driveStopLatched === null ? t("等待底盘锁止状态")
                    : running ? t("任务接管中") : t("系统空闲 · 可手动操作")}
            </span>
          </div>
        </div>
        <div className="console-grid">
          <section className="console-panel navigation-panel">
            <div className="panel-heading"><span>01</span><div><strong>{t("定位与导航")}</strong>
              <small>{t("从已验证的场地地点中选择目标")}</small></div></div>
            <div className="destination-row">
              <label><span>{t("目标地点")}</span><select value={placeId}
                onChange={event => setPlaceId(event.target.value)} disabled={!places.length}>
                {places.map(place => <option key={place.id} value={place.id}>
                  {t(placeNames[place.id]) || place.id} · {place.id}
                </option>)}
              </select></label>
              <button className="primary destination-action" onClick={() => manual('navigate')}
                disabled={manualBusy || !!running || driveStopLatched !== false || !placeId}>
                {t("开始导航 ")}</button>
            </div>
            <div className="panel-foot">
              <span>{mapping?.pose ? t("当前位置 {0}, {1} m", mapping.pose.x.toFixed(2), mapping.pose.y.toFixed(2))
                : t("当前位置尚未建立")}</span>
              <button className="text-action" onClick={() => manual('localize')}
                disabled={manualBusy || !!running || driveStopLatched !== false}>
                {t("重新定位 ")}</button>
            </div>
          </section>
          <section className="console-panel joystick-panel">
            <div className="panel-heading"><span>02</span><div><strong>{t("底盘微调")}</strong>
              <small>{t("近距离观察下的低速点动")}</small></div></div>
            <BaseJoystick
              disabled={manualBusy || !!running || driveStopLatched !== false}
              onError={onError}
            />
          </section>
          <section className="console-panel posture-panel">
            <div className="panel-heading"><span>03</span><div><strong>{t("机器人姿态")}</strong>
              <small>{t("调用已验证的固定姿态")}</small></div></div>
            <div className="preset-list">
              <button onClick={() => manual('arm_ready')}
                disabled={manualBusy || !!running || driveStopLatched !== false}>
                <span>{t("右臂回 Ready")}</span><small>{t("恢复携带姿态")}</small></button>
              <button onClick={() => manual('head_ready')}
                disabled={manualBusy || !!running || driveStopLatched !== false}>
                <span>{t("头部向前")}</span><small>{t("回到水平正前方")}</small></button>
              <button onClick={() => manual('gripper_open')}
                disabled={manualBusy || !!running || driveStopLatched !== false}>
                <span>{t("打开右夹爪")}</span><small>{t("释放当前夹持")}</small></button>
            </div>
          </section>
        </div>
        {manualResult && <div className="operation-result" role="status"><span>{t("操作完成")}</span>{t(manualResult)}</div>}
      </article>
    </section>}
    <section className="lower-grid">
      <HealthPanel health={health} />
      <HistoryPanel tasks={history} />
    </section>
  </div>
}

function CameraPreview({ camera, perception }: {
  camera: 'head' | 'wrist', perception: PerceptionState | null,
}) {
  const { t } = useLanguage()
  const detected = camera === 'head' && perception !== null
  const source = `/api/v1/cameras/${camera}/stream`
  const label = camera === 'head' ? t("头部相机") : t("右腕相机")
  return <figure className={detected ? 'detection-active' : ''}>
    <img src={source} alt={t("{0}{1}", label, detected ? t("识别帧") : t("实时预览"))} />
    {detected && <>
      <svg className={`bbox-overlay ${perception.kind}`}
        viewBox={`0 0 ${perception.image_width} ${perception.image_height}`}
        preserveAspectRatio="xMidYMid meet" aria-hidden="true">
        <rect x={perception.bbox[0]} y={perception.bbox[1]}
          width={Math.max(1, perception.bbox[2] - perception.bbox[0])}
          height={Math.max(1, perception.bbox[3] - perception.bbox[1])} />
      </svg>
      <div className="detection-result">
        {perception.kind === 'person' ? t("人员") : perception.label}
        <span>{Math.round(perception.confidence * 100)}%</span>
      </div>
    </>}
    <figcaption><strong>{detected ? t("头部识别帧") : label}</strong>
      <span>{camera === 'head'
        ? detected ? t("与当前三维目标对应") : t("D455 · 场景与人员")
        : t("抓取近景 · MJPG")}</span></figcaption>
  </figure>
}

export function BaseJoystick({ disabled, onError }: {
  disabled: boolean, onError: (message: Message) => void,
}) {
  const { t } = useLanguage()
  const { armed, setArmed, move } = useBaseTeleop(disabled, onError)
  return <div className="joystick-control">
    <button className={armed ? 'joystick-lock unlocked' : 'joystick-lock'}
      aria-pressed={armed} disabled={disabled} onClick={() => setArmed(!armed)}>
      <i />{armed ? t("结束遥控") : disabled ? t("任务中已锁定") : t("解锁遥控")}
    </button>
    <JoystickPad size={142} disabled={!armed || disabled}
      onMove={(linear, angular) => move(linear * .10, angular * .40)}
      onRelease={() => move(0, 0)} />
  </div>
}

function StatusMetric({ label, value, state }: {
  label: string, value: string, state: 'ok' | 'warn' | 'bad' | 'muted'
}) {
  return <div className="status-metric"><span className={`status-dot ${state}`} />
    <div><small>{label}</small><strong>{value}</strong></div></div>
}

function TaskProgress({ task }: { task: Task }) {
  const { t, s } = useLanguage()
  const percentage = Math.round(Math.max(0, Math.min(1, task.progress)) * 100)
  return <div className="task-progress">
    <div className="task-meta"><span>{t(capabilityNames[task.current_capability])
      || task.current_capability || task.status}</span><strong>{percentage}%</strong></div>
    <div className="bar"><span style={{ width: `${percentage}%` }} /></div>
    <p>{task.phase}{task.message ? ` · ${s(task.message)}` : ''}
      {t(" · {0} · 本阶段 {1} s", task.grasp_backend_used || task.grasp_backend, task.stage_elapsed_s.toFixed(1))}</p>
    {task.error_code !== 0 && <p className="error-detail">{t("错误 ")}{task.error_code}: {s(task.message)}</p>}
  </div>
}

function HealthPanel({ health }: { health: Health }) {
  const { t, s } = useLanguage()
  const diagnostics = useMemo(() => Object.entries(health.diagnostics)
    .sort((left, right) => right[1].level - left[1].level), [health])
  const warnings = diagnostics.filter(([, item]) => item.level === 1).length
  const errors = diagnostics.filter(([, item]) => item.level >= 2).length
  return <article>
    <div className="card-heading"><div><p className="section-label">SYSTEM HEALTH</p>
      <h2>{t("系统诊断")}</h2></div>
      <span className={`health-count ${errors ? 'bad' : warnings ? 'warn' : 'ok'}`}>
        {errors ? t("{0} 项故障", errors) : warnings ? t("{0} 项关注", warnings) : t("全部正常")}
      </span></div>
    <div className="diagnostic-overview">
      <StatusMetric label={t("任务执行")} value={health.execute_task_available ? t("服务可用") : t("服务不可用")}
        state={health.execute_task_available ? 'ok' : 'bad'} />
      <StatusMetric label={t("导航传感")} value={t("激光雷达 + 轮速里程计")} state="ok" />
    </div>
    <div className="diagnostic-list">
      {diagnostics.map(([name, item]) => <details className={`diagnostic level-${item.level}`} key={name}>
        <summary><span className={`status-dot ${item.level >= 2 ? 'bad' : item.level === 1 ? 'warn' : 'ok'}`} />
          <div><strong>{t(diagnosticName(name))}</strong><small>{s(diagnosticMessage(item.message))}</small></div>
          <em>{item.level >= 2 ? t("故障") : item.level === 1 ? t("关注") : t("正常")}</em></summary>
        <div className="diagnostic-detail">
          <p>{s(item.message) || t("该组件未提供补充说明")}</p>
          {item.hardware_id && <p><b>{t("硬件标识")}</b>{item.hardware_id}</p>}
          {Object.keys(item.values).length > 0 && <dl>{Object.entries(item.values).map(([key, value]) =>
            <div key={key}><dt>{key}</dt><dd>{value}</dd></div>)}</dl>}
        </div>
      </details>)}
      {diagnostics.length === 0 && <p className="empty-state">{t("尚未收到系统诊断消息")}</p>}
    </div>
  </article>
}

function diagnosticName(name: string) {
  if (name.includes('scan_map_consistency')) return '激光-地图一致性'
  if (name.includes('Controllers Activity')) return '运动控制器'
  if (name.includes('Controller Manager Activity')) return '控制运行时'
  if (name.includes('Hardware Components')) return '左右共享总线'
  if (name.includes('localization')) return '地图定位'
  if (name.includes('navigation')) return '路径导航'
  if (name.includes('drive_safety')) return '底盘指令通道'
  if (name.includes('startup_ready')) return '启动姿态'
  return name.replace(/^xlerobot\//, '')
}

function diagnosticMessage(message: string) {
  const scanMap = message.match(/^scan-map score=([0-9.]+) \(minimum=([0-9.]+)\)$/)
  if (scanMap) return msg('当前匹配分 {0}，最低要求 {1}', scanMap[1], scanMap[2])
  if (message === 'warming scan-map score window') return '正在累积激光-地图匹配样本'
  if (message.startsWith('too few usable map endpoints')) return '可用激光地图端点不足'
  if (message.startsWith('missing timestamped laser-to-map TF')) return '缺少扫描时刻的激光到地图 TF'
  if (message === 'Controller Manager is running') return '控制循环正在运行'
  if (message === 'All hardware components are active') return '两条硬件总线均已激活'
  if (message === 'Managed nodes are active') return '相关 Nav2 节点均已激活'
  if (message === 'drive command path ready') return '底盘命令链路就绪'
  if (message.includes('ready pose')) return '头部、右臂和夹爪已在 Ready 位'
  if (message.includes('High execution jitter')) return '控制器执行时间曾出现短时波动，请展开查看平均值和峰值'
  return message || '状态正常'
}

function HistoryPanel({ tasks }: { tasks: Task[] }) {
  const { t } = useLanguage()
  return <article>
    <p className="section-label">RECENT RUNS</p>
    <h2>{t("最近任务")}</h2>
    <div className="history">
      {tasks.length === 0 && <p>{t("尚无任务记录")}</p>}
      {tasks.slice(0, 6).map(task => <div key={task.task_id}>
        <span>{task.object_id} · {task.grasp_backend_used || task.grasp_backend}</span><span>{t(capabilityNames[task.current_capability]) || task.current_capability || '—'}</span>
        <strong className={task.status.toLowerCase()}>{task.status === 'SUCCEEDED' ? t("完成")
          : task.status === 'FAILED' ? t("失败") : task.status === 'CANCELED' ? t("已取消") : task.status}</strong>
      </div>)}
    </div>
  </article>
}
