import { useLanguage, type Message } from './i18n'
import { useEffect, useState } from 'react'

type Arrival = { target: string, report_id: string, target_xyz_mm: number[], observed_xyz_mm: number[],
  planar_error_mm: number, height_mm: number, height_shortfall_mm: number, max_joint_error_deg: number }
type Report = { id: string, status: string, message: string, updated_at: string, source: string,
  independent_arrivals: number, frames: number, arrivals: Arrival[],
  summary?: { mean_planar_error_mm: number, mean_height_shortfall_mm: number },
  suggestion?: { frame: string, add_to_target_xyz_m: number[], raise_target_mm: number,
    between_pose_error_std_mm: number[], advice: string } | null }
const names: Record<string, string> = { center: '中心', right: 'X +30', left: 'X −30' }
const number = (value: number) => value.toFixed(1)

export function HoverReport() {
  const { t, s } = useLanguage()
  const [data, setData] = useState<{ report: Report | null, stale: boolean } | null>(null)
  const [error, setError] = useState<Message>('')
  useEffect(() => {
    let disposed = false
    let timer: ReturnType<typeof setTimeout>
    const poll = async () => {
      try {
        const response = await fetch('/workbench-api/hover-result')
        if (!response.ok) throw new Error("暂时无法读取已保存结果")
        const result = await response.json()
        if (!disposed) { setData(result); setError('') }
      } catch (reason) { if (!disposed) setError(String(reason)) }
      if (!disposed) timer = setTimeout(poll, 1500)
    }
    void poll()
    return () => { disposed = true; clearTimeout(timer) }
  }, [])
  const report = data?.report
  if (!report) return <section className="engineering-card"><h2>{t("悬停验证结果")}</h2>
    <p>{s(error) || t("尚无三点验证结果。点击自动运行，机器人会依次测量三个点并生成建议。")}</p></section>
  const suggestion = report.suggestion
  const usable = Boolean(suggestion && !data?.stale && !error && report.status === 'COMPLETED')
  const download = () => {
    const blob = new Blob([JSON.stringify({ ...suggestion, run_id: report.id, applied: false }, null, 2)], { type: 'application/json' })
    const url = URL.createObjectURL(blob), link = document.createElement('a')
    link.href = url; link.download = `hover-offset-${report.id}.json`; link.click()
    setTimeout(() => URL.revokeObjectURL(url), 1000)
  }
  return <section className="engineering-card hover-report">
    <div className="hover-report-heading"><div><p className="eyebrow">HOVER · RESULT</p><h2>{t("悬停验证结果")}</h2></div>
      <strong>{report.status === 'COMPLETED' ? t("测量完成 · 补偿尚未验证") : report.status === 'RUNNING' ? t("自动测量中") : t("测量中断 · 已完成数据保留")}</strong></div>
    <p>{s(report.message)}</p>
    {data?.stale && <p className="notice" role="alert">{t("标定版本已变化：以下是历史结果，建议补偿不可沿用，请重新测量。")}</p>}
    {error && <p role="alert">{s(error)}{t("；当前显示缓存结果。")}</p>}
    <div className="hover-summary-grid">
      <div><small>{t("独立到位")}</small><strong>{report.independent_arrivals} / 3</strong><span>{report.frames} {t(" 帧，不等于 ")}{report.frames} {t(" 次重复实验")}</span></div>
      <div><small>{t("平均平面偏差")}</small><strong>{report.summary ? number(report.summary.mean_planar_error_mm) : '—'} <em>mm</em></strong></div>
      <div><small>{t("平均高度不足（正值偏低）")}</small><strong>{report.summary ? number(report.summary.mean_height_shortfall_mm) : '—'} <em>mm</em></strong></div>
    </div>
    {report.arrivals.length > 0 && <>
      <div className="hover-charts">
        <figure><figcaption>{t("俯视对齐 · ○ 目标 / ● 实测（板坐标，mm）")}</figcaption>
          <svg viewBox="0 0 400 190" role="img" aria-label={t("三个悬停点的目标与实际平面位置")}>
            <path d="M30 100 H370 M200 25 V165" stroke="#526775" strokeDasharray="4 4" />
            <text x="355" y="90">X</text><text x="210" y="30">Y</text>
            {report.arrivals.map(row => <g key={row.target}>
              <line x1={200 + row.target_xyz_mm[0] * 3} y1={100 - row.target_xyz_mm[1] * 3}
                x2={200 + row.observed_xyz_mm[0] * 3} y2={100 - row.observed_xyz_mm[1] * 3} stroke="#f0ba62" strokeWidth="2" />
              <circle cx={200 + row.target_xyz_mm[0] * 3} cy={100 - row.target_xyz_mm[1] * 3} r="8" fill="none" stroke="#55d7bc" strokeWidth="2" />
              <circle cx={200 + row.observed_xyz_mm[0] * 3} cy={100 - row.observed_xyz_mm[1] * 3} r="5" fill="#f0ba62" />
              <text x={200 + row.target_xyz_mm[0] * 3} y="160" textAnchor="middle">{t(names[row.target])}</text>
            </g>)}
          </svg></figure>
        <figure><figcaption>{t("侧视高度 · 虚线为目标 200 mm")}</figcaption>
          <svg viewBox="0 0 400 190" role="img" aria-label={t("三个点的目标高度与实测高度")}>
            <path d="M30 40 H370" stroke="#55d7bc" strokeDasharray="5 4" /><text x="30" y="29">200 mm</text>
            {report.arrivals.map((row, i) => <g key={row.target}>
              <rect x={70 + i * 110} y={160 - row.height_mm * .6} width="40" height={Math.max(0, row.height_mm * .6)} rx="4" fill="#f0ba62" />
              <text x={90 + i * 110} y={150 - row.height_mm * .6} textAnchor="middle">{number(row.height_mm)}</text>
              <text x={90 + i * 110} y="181" textAnchor="middle">{t(names[row.target])}</text>
            </g>)}
          </svg></figure>
      </div>
      <table><thead><tr><th>{t("点位")}</th><th>{t("平面偏差")}</th><th>{t("实际高度")}</th><th>{t("高度不足")}</th><th>{t("最大关节到位偏差")}</th></tr></thead><tbody>
        {report.arrivals.map(row => <tr key={row.target}><td>{t(names[row.target])}</td><td>{number(row.planar_error_mm)} mm</td>
          <td>{number(row.height_mm)} mm</td><td>{number(row.height_shortfall_mm)} mm</td><td>{number(row.max_joint_error_deg)}°</td></tr>)}
      </tbody></table>
    </>}
    <div className="hover-suggestion"><h3>{t("建议补偿 · 仅作为下一轮验证初值")}</h3>
      {usable && suggestion ? <>
        <p>{t("建议目标高度")}{suggestion.raise_target_mm >= 0 ? t("增加") : t("降低")} <strong>{number(Math.abs(suggestion.raise_target_mm))} mm</strong>{t("。 板 X 偏移 ")}<strong>{(suggestion.add_to_target_xyz_m[0] * 1000).toFixed(1)} mm</strong>{t("， 板 Y 偏移 ")}<strong>{(suggestion.add_to_target_xyz_m[1] * 1000).toFixed(1)} mm</strong>。</p>
        <pre>{`frame: calibration_board\nadd_to_target_xyz_m: [${suggestion.add_to_target_xyz_m.map(v => v.toFixed(5)).join(', ')}]\napplied: false`}</pre>
        <p>{t("使用方式：新目标 = 原目标 + 上述偏移。板 Z 朝下，因此负 Z 表示抬高；不是机器人 base_link 的 XYZ 参数。")}</p>
        <p>{s(suggestion.advice)}</p>
        <p>{t("三点之间的偏差标准差（X / Y / Z）：")}{suggestion.between_pose_error_std_mm.map(number).join(' / ')} {t(" mm。 这是姿态间变化，不是补偿后的实测精度；常量偏移可能无法消除全部误差。")}</p>
        <button onClick={download}>{t("下载建议参数（不应用）")}</button>
      </> : <p>{data?.stale ? t("历史建议已失效。") : t("完成同一标定版本、同一桌面场景的三点测量后生成建议。")}</p>}
    </div>
    <p>{t("这是同相机视觉计量，不是独立尺量真值。不把总误差直接归因为重力下垂，不改写舵机零位、手眼矩阵或 Demo 生效配置。")}</p>
    <small>{t("测量记录：")}{report.id} · {report.updated_at}{report.source === 'imported_previous_run' ? t(" · 来自上轮已完成的三点实测") : ''}</small>
  </section>
}
