import { useLanguage, msg, type Message } from './i18n'
import { useEffect, useRef, useState } from 'react'
import {
  api, type ServoCalibrationCommand, type ServoCalibrationGroup,
  type ServoCalibrationJoint, type ServoCalibrationState,
} from './api'

const groups: Array<{ id: ServoCalibrationGroup, title: string, subtitle: string }> = [
  { id: 'right_arm', title: '右臂', subtitle: '5 个关节 + 夹爪' },
  { id: 'left_arm', title: '左臂', subtitle: '5 个关节 + 夹爪' },
  { id: 'head', title: '头部', subtitle: '水平转动 + 俯仰' },
  { id: 'leader', title: 'Leader 主臂', subtitle: '独立示教臂 · 6 个舵机' },
]
const jointNames: Record<string, string> = {
  shoulder_pan: '肩部水平', shoulder_lift: '肩部抬升', elbow_flex: '肘部',
  wrist_flex: '手腕俯仰', wrist_roll: '手腕旋转', gripper: '夹爪',
  pan: '水平转动', tilt: '俯仰',
}
const armJoints = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
const rawValue = (value: number | undefined) =>
  value !== undefined && Number.isFinite(value) && value >= 0 ? String(value) : '—'
const isGroup = (value: string): value is ServoCalibrationGroup =>
  groups.some(group => group.id === value)
const healthyMessages: Record<string, string> = {
  'not read since startup': '尚未读取',
  'zero pose has not been captured': '在线 · 待零位',
  'ready to record group range': '零位已记录 · 待采集',
  'range captured': '本组已完成',
  'range sufficient': '范围已达标',
}
const warningMessages: Record<string, string> = {
  'include the zero pose in the recorded range': '范围还未覆盖零位，请手动经过零位',
  'move this joint further: coverage below 60%': '继续轻轻活动，覆盖尚未达到 60%',
  'servo read failed; check the cable and retry': '读取失败，请检查线缆后重试',
  'encoder value outside 0..4095': '编码器读数异常：超出 0–4095',
  'invalid encoder range; reset this group and inspect the joint': '编码器范围异常：暂停后重置本组并检查关节',
  'encoder wrapped or jumped >2048 ticks; reset this group; do not cross encoder zero':
    '编码器跨零 / 跳变：暂停后重置本组，避免跨越编码器零点',
}
function rangeReady(joint: ServoCalibrationJoint | undefined): boolean {
  return Boolean(joint && joint.online && joint.zero_captured && joint.raw_min >= 0
    && joint.raw_min <= joint.zero && joint.zero <= joint.raw_max && joint.coverage >= .6
    && (!joint.message || joint.message === 'range sufficient' || joint.message === 'range captured'))
}

export function ServoCalibrationWorkspace({ unitId, captureOnly, onError, onRestart, restartBusy = false }: {
  unitId: string, captureOnly: boolean, onError: (message: Message) => void,
  onRestart?: () => void, restartBusy?: boolean,
}) {
  const { t, s } = useLanguage()
  const [state, setState] = useState<ServoCalibrationState | null>(null)
  const [group, setGroup] = useState<ServoCalibrationGroup>('right_arm')
  const [busy, setBusy] = useState<Message>('')
  const [readError, setReadError] = useState<Message>('')
  const [actionError, setActionError] = useState<Message>('')
  const [notice, setNotice] = useState<Message>('')
  const [resetConfirm, setResetConfirm] = useState(false)
  const [unchangedHardware, setUnchangedHardware] = useState(false)
  const busyRef = useRef(false)
  const revision = useRef(0)
  const mounted = useRef(false)
  const initialized = useRef(false)
  const onErrorRef = useRef(onError)
  onErrorRef.current = onError

  const acceptStatus = (value: ServoCalibrationState) => {
    setState(value)
    if (!initialized.current && value.joints.some(joint => joint.name.startsWith('leader.'))) setGroup('leader')
    // Restore a running/paused group on first load. Ordinary status updates must
    // never jump the user's selected tab or remount the controls.
    if ((!initialized.current || value.phase === 'RANGE_RECORDING')
      && isGroup(value.active_group)) setGroup(value.active_group)
    initialized.current = true
  }

  useEffect(() => {
    mounted.current = true
    let disposed = false
    let timer: ReturnType<typeof setTimeout>
    const poll = async () => {
      const startedAtRevision = revision.current
      if (!busyRef.current) {
        try {
          const value = await api.servoCalibrationStatus()
          if (!disposed && startedAtRevision === revision.current && !busyRef.current) {
            acceptStatus(value)
            setReadError('')
          }
        } catch (reason) {
          if (!disposed && startedAtRevision === revision.current && !busyRef.current) {
            setReadError(String(reason))
          }
        }
      }
      // Schedule after completion, not setInterval: slow reads cannot overlap.
      if (!disposed) timer = setTimeout(poll, 500)
    }
    void poll()
    return () => { disposed = true; mounted.current = false; clearTimeout(timer) }
  }, [unitId])

  const run = async (commands: ServoCalibrationCommand[], label: Message) => {
    if (busyRef.current) return
    busyRef.current = true
    revision.current += 1
    setBusy(label)
    setActionError('')
    setNotice('')
    try {
      for (const command of commands) {
        const value = await api.servoCalibrationStep(unitId, command, group)
        if (!mounted.current) return
        acceptStatus(value)
      }
      setReadError('')
      if (commands.includes('finish_range')) setNotice(msg("本组已完成。选择下一组继续，已有采集数据会保留。"))
      if (commands.includes('pause_range')) setNotice(msg("已暂停范围记录；已采集的零位和范围保留，扭矩状态不变。"))
      if (commands.includes('reset_group')) {
        setNotice(msg("已清空本组的本次采集数据。其他组、已有标定和舵机设置未修改。"))
        setResetConfirm(false)
        setUnchangedHardware(false)
      }
    } catch (reason) {
      if (mounted.current) {
        setActionError(String(reason))
        onErrorRef.current(String(reason))
      }
    } finally {
      busyRef.current = false
      if (mounted.current) setBusy('')
    }
  }

  const selected = groups.find(item => item.id === group)!
  const leaderOnly = state?.joints.some(joint => joint.name.startsWith('leader.')) ?? false
  const availableGroups = groups.filter(item => leaderOnly ? item.id === 'leader' : item.id !== 'leader')
  const names = group === 'head' ? ['pan', 'tilt'] : armJoints
  const rows = names.map(name => ({
    name, joint: state?.joints.find(joint => joint.name === `${group}.${name}`),
  }))
  const recording = state?.phase === 'RANGE_RECORDING'
  const finalized = state?.phase === 'FINALIZED'
  const completed = state?.completed_groups.includes(group) ?? false
  const released = state?.released_groups?.includes(group) ?? false
  const hasZero = rows.every(({ joint }) => joint?.zero_captured)
  const online = rows.every(({ joint }) => joint?.online)
  const hasReference = rows.every(({ joint }) =>
    joint && Number.isFinite(joint.reference_zero) && joint.reference_zero >= 0)
  const enoughRange = rows.every(({ joint }) => rangeReady(joint))
  const allComplete = availableGroups.every(item => state?.completed_groups.includes(item.id))
  const disabled = Boolean(busy) || !state || Boolean(readError) || finalized
  const step = completed ? 3 : recording || hasZero ? 2 : released ? 1 : 0

  return <section className="engineering-card servo-workspace">
    <header className="servo-heading">
      <div><p className="section-label">SERVO CALIBRATION · {unitId}</p>
        <h2>{t("整组标定，一次摆动全部关节")}</h2>
        <p className="card-intro">{t("选一组 → 托住并释放 → 确认零位 → 手动活动整组。无需逐关节切换或反复点击。")}</p>
      </div>
      <span className={`servo-phase ${recording ? 'recording' : ''}`}>
        {finalized ? t("已保存") : recording ? t("正在采集") : state?.phase === 'PAUSED' ? t("已暂停")
          : state?.phase === 'ZERO_CAPTURED' ? t("零位已确认") : state?.phase === 'RANGE_CAPTURED' ? t("本组完成")
            : state ? t("待操作") : t("连接中")}
      </span>
    </header>

    {onRestart && <div className="servo-reset">
      <button disabled={Boolean(busy) || recording || restartBusy} onClick={onRestart}>
        {restartBusy ? t("正在归档并重新开始…") : leaderOnly ? t("重新开始 Leader 标定") : t("重新开始从臂 / 头部标定")}
      </button>
      <p>{t("归档本轮全部记录并开始新一轮，不删除旧结果，不改变生效标定，不自动上力或移动。 ")}{recording ? t("请先暂停范围采集。") : ''}</p>
    </div>}
    {finalized && <p className="notice">{t("本轮已保存。单组重采仅用于尚未保存的采集；要重新标定，请使用上方“重新开始”按钮。 ")}{!onRestart && t("独立工具请退出后使用 --fresh 重新启动。")}</p>}

    <div className="servo-groups" role="group" aria-label={t("标定分组")}>
      {availableGroups.map((item, index) => <button key={item.id}
        className={`servo-group ${group === item.id ? 'selected' : ''}`}
        aria-pressed={group === item.id}
        disabled={Boolean(busy) || (recording && group !== item.id)}
        onClick={() => { setGroup(item.id); setResetConfirm(false); setUnchangedHardware(false); setActionError(''); setNotice('') }}>
        <span className="servo-group-number">{state?.completed_groups.includes(item.id) ? '✓' : `0${index + 1}`}</span>
        <span><strong>{t(item.title)}</strong><small>{t(item.subtitle)}</small></span>
        <em>{state?.completed_groups.includes(item.id) ? t("已完成") : recording && state.active_group === item.id ? t("采集中") : t("待完成")}</em>
      </button>)}
    </div>

    {leaderOnly && <p className="card-intro">{t("Leader 独立标定 · 尚未实机验收。仅连接示教臂，不使用从臂零位；结果单独保存，不替换机器人标定。")}</p>}
    {readError && <div className="servo-error" role="alert">
      <strong>{t("暂时无法读取舵机状态")}</strong><p>{t("显示的是最后一次读数，操作按钮暂不可用。正在自动重试，不会清空已采集数据。")}</p><code>{s(readError)}</code>
    </div>}
    {actionError && <div className="servo-error" role="alert">
      <strong>{t("这一步未完成")}</strong><p>{recording ? t("范围仍在记录，请按提示补充活动后再次完成；不要重置已有数据。") : t("已有数据保留，请检查下面的原因再重试。")}</p><code>{s(actionError)}</code>
    </div>}

    <div className="servo-main-grid">
      <div className="servo-steps">
        <div className={`servo-step ${step === 0 ? 'current' : ''}`}>
          <span className="servo-step-index">1</span><div>
            <h3>{t("托住")}{t(selected.title)}{t("，释放本组扭矩")}</h3>
            <p>{group === 'head' ? t("扶住头部，避免失去支撑后下垂。") : t("先托稳机械臂、清空夹爪，让机械臂可以由手轻轻带动。")}{t("仅影响所选组，不操作轮子。")}</p>
            <button disabled={disabled || recording || completed}
              onClick={() => run(['release_torque'], msg("正在释放本组扭矩…"))}>
              {released ? t("已释放 · 再次释放本组") : t("已托住，释放本组扭矩")}
            </button>
          </div>
        </div>
        <div className={`servo-step ${step === 1 ? 'current' : ''}`}>
          <span className="servo-step-index">2</span><div>
            <h3>{t("确认整组零位，开始记录")}</h3>
            {!hasZero && <>
              <p>{t("整组摆到 ")}<strong>{t("URDF 对应的机械零位")}</strong>{t("，保持静止再记录。不是 ready 姿态，也不是任意中间姿态。")}</p>
              <details className="servo-zero-reference">
                <summary>{t("查看机械零位参考（不是 ready）")}</summary>
                <a href="/calibration-zero.png" target="_blank" rel="noreferrer">
                  <img src="/calibration-zero.png" alt={t("URDF 机械零位参考：正面与右侧视图，所有关节 q=0，不是 ready 姿态")} />
                </a><p>{t("按真实 URDF 渲染，点击可放大。仅作摆位参考，不会驱动机器人。")}</p>
              </details>
              <button className={released ? 'primary' : ''}
                disabled={disabled || !released || !online || recording || completed}
                onClick={() => run(['capture_zero', 'start_range'], msg("正在记录零位并启动整组采集…"))}>
                {t("记录零位并开始整组采集 ")}</button>
              {hasReference && <details className="servo-existing-zero">
                <summary>{t("未拆装过？也可以保留已有标定零位")}</summary>
                <p>{t("沿用已有有效标定的零位，仅重新采集活动范围。它不是本次重新测得的零位。")}</p>
                <label><input type="checkbox" checked={unchangedHardware}
                  disabled={disabled || recording || completed}
                  onChange={event => setUnchangedHardware(event.target.checked)} />{t("未拆装舵机 / 未更改硬件零偏")}</label>
                <button disabled={disabled || !released || !online || !unchangedHardware || recording || completed}
                  onClick={() => run(['use_existing_zero', 'start_range'], msg("正在保留已有零位并启动整组采集…"))}>
                  {t("保留已有零位并开始采集 ")}</button>
              </details>}
            </>}
            {hasZero && <p className="servo-check">{t("✓ 本组零位已记录，不必再摆一次。")}{rows.some(({ joint }) => joint?.zero_source?.startsWith('existing:')) ? t("来源：沿用已有有效标定。") : t("来源：本次记录。")}</p>}
          </div>
        </div>
        <div className={`servo-step ${step === 2 ? 'current' : ''}`}>
          <span className="servo-step-index">3</span><div>
            <h3>{t("手动活动整组，然后完成")}</h3>
            <p>{t("按方便的顺序活动每个关节")}{group !== 'head' ? t("，包括打开、合拢夹爪") : ''}{t("。所有关节同时记录；覆盖条达到 60% 即达标，不要用力顶机械限位。")}</p>
            {recording ? <div className="servo-actions">
              <button className="primary" disabled={disabled}
                onClick={() => run(['finish_range'], msg("正在检查本组范围…"))}>{t("完成本组")}</button>
              <button disabled={Boolean(busy)}
                onClick={() => run(['pause_range'], msg("正在暂停范围记录…"))}>{t("暂停采集")}</button>
            </div> : hasZero && !completed && !finalized ?
              <button className="primary" disabled={disabled || !released || !online}
                onClick={() => run(['start_range'], msg("正在继续整组采集…"))}>{t("继续采集（保留已有范围）")}</button>
              : <p className="servo-check">{completed ? t("✓ 本组已完成，请选择下一组。") : t("确认零位后会自动开始记录。")}</p>}
            {recording && <p className="servo-range-hint">{enoughRange ? t("所有关节范围已达标，可以完成本组。") : t("查看实时读数中的提示：补足活动范围并覆盖零位；出现编码器异常先暂停处理。")}</p>}
            {hasZero && !released && !recording && !completed && !finalized && <p className="servo-range-hint">{t("工具重启后，请先重新托住并释放本组扭矩，再继续采集。")}</p>}
          </div>
        </div>
      </div>

      <div className="servo-readings">
        <div className="servo-readings-heading"><div><h3>{t(selected.title)} {t(" · 实时读数")}</h3><p>{t("RAW TICKS · 500 ms 刷新")}</p></div>
          <button disabled={disabled || recording}
            onClick={() => run(['scan'], msg("正在扫描舵机…"))}>{t("重新读取")}</button></div>
        <div className="servo-table-scroll"><table className="servo-table">
          <thead><tr><th>{t("关节 / ID")}</th><th>{t("当前位置")}</th><th>{t("零位")}</th><th>{t("已采集范围")}</th><th>{t("覆盖")}</th></tr></thead>
          <tbody>{rows.map(({ name, joint }) => <JointRow key={`${group}.${name}`} name={name} joint={joint} />)}</tbody>
        </table></div>
        <p className="hint">{t("“—” 表示尚未获得读数。读数失败或跨越编码器边界会在对应行提示；请先处理提示，再完成本组。")}</p>
        <div className="servo-reset">
          {resetConfirm ? <div role="alertdialog" aria-label={t("重置{0}本次采集", t(selected.title))}>
            <strong>{t("只清空")}{t(selected.title)}{t("的本次采集？")}</strong>
            <p>{t("本组本次记录的零位、范围和完成状态将清空；其他组、已激活标定和舵机设置不变，不会驱动机械臂。")}</p>
            <div className="servo-actions"><button className="danger" disabled={disabled || recording}
              onClick={() => run(['reset_group'], msg("正在重置本组采集…"))}>{t("确认重置本组")}</button>
              <button disabled={Boolean(busy)} onClick={() => setResetConfirm(false)}>{t("取消")}</button></div>
          </div> : <button className="text-action" disabled={disabled || recording}
            onClick={() => setResetConfirm(true)}>{t("重新采集这一组…")}</button>}
        </div>
      </div>
    </div>

    <footer className="servo-footer">
      <div><strong>{state?.completed_groups.length ?? 0} / {availableGroups.length} {t(" 组完成")}</strong>
        <p>{t("采集进度自动保存在机器人上，刷新页面不丢失；重启工具后可继续。完成采集只保存本机结果，不自动上力，不替换 active 标定。")}</p>
      </div>
      <button className="primary" disabled={disabled || recording || !allComplete}
        onClick={() => run(['finalize'], msg("正在保存舵机标定结果…"))}>
        {finalized ? t("标定结果已保存") : captureOnly ? t("完成采集并保存结果") : t("验收全部并写入 draft")}
      </button>
    </footer>
    <p className="servo-operation" role="status">{t(busy || notice)}</p>
    {state?.session_uri && <p className="servo-file">{t("进度文件 ")}<code>{state.session_uri}</code></p>}
    {state?.result_uri && <p className="saved">{t("标定结果 ")}<code>{state.result_uri}</code></p>}
  </section>
}

function JointRow({ name, joint }: { name: string, joint?: ServoCalibrationJoint }) {
  const { t, s } = useLanguage()
  const coverage = joint && Number.isFinite(joint.coverage) ? Math.max(0, joint.coverage) : 0
  const warning = joint && ((!joint.online && joint.message !== 'not read since startup')
    || (joint.message && !healthyMessages[joint.message]))
  const status = !joint ? t("等待数据") : joint.message
    ? healthyMessages[joint.message] || warningMessages[joint.message] || joint.message
    : !joint.online ? t("读取失败") : joint.range_captured ? t("已完成") : joint.zero_captured ? t("零位已记录") : t("在线 · 待零位")
  return <tr className={warning ? 'servo-joint-warning' : ''}>
    <th scope="row"><strong>{t(jointNames[name])}</strong><small>{name} · ID {joint?.servo_id ?? '—'}</small>
      <span title={s(joint?.message)}>{s(status)}</span></th>
    <td>{rawValue(joint?.position)}</td>
    <td>{rawValue(joint?.zero)}<small>{joint?.zero_source?.startsWith('existing:') ? t("沿用已有") : joint?.zero_captured ? t("本次记录") : ''}</small>
      {!joint?.zero_captured && joint && joint.reference_zero >= 0 && <small>{t("已有 ")}{joint.reference_zero}</small>}</td>
    <td>{rawValue(joint?.raw_min)} → {rawValue(joint?.raw_max)}</td>
    <td><div className={`servo-coverage ${rangeReady(joint) ? 'enough' : ''}`} role="progressbar"
      aria-label={t("{0}范围覆盖", t(jointNames[name]))} aria-valuemin={0} aria-valuemax={100} aria-valuenow={Math.round(Math.min(1, coverage) * 100)}>
      <span style={{ width: `${Math.min(1, coverage) * 100}%` }} /><i /></div><small>{Math.round(coverage * 100)}% / 60%</small></td>
  </tr>
}
