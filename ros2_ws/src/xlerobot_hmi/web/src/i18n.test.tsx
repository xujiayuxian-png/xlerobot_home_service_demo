import { useState } from 'react'
import { act, cleanup, fireEvent, render, screen } from '@testing-library/react'
import { afterEach, beforeEach, expect, it, vi } from 'vitest'
import { LanguageProvider, LanguageToggle, msg, translate, translateService, useLanguage } from './i18n'

beforeEach(() => { window.localStorage.clear() })
afterEach(() => { cleanup(); vi.restoreAllMocks() })

function Example() {
  const { t } = useLanguage()
  const [notice, setNotice] = useState(msg('已记录 {0} 的位置和朝向。', '地图-机器人-{0}'))
  return <><h1>{t('场地地图')}</h1>
    <input aria-label={t('物体名')} placeholder={t('物体标签')} defaultValue="地图机器人" />
    <img alt={t('头部相机')} title={t('实时预览')} />
    <p>{t(notice)}</p><button onClick={() => setNotice(msg('已删除 {0}。', '地图-1'))}>notice</button>
    <code>/example/地图/机器人.yaml</code><LanguageToggle /></>
}
const english = () => fireEvent.click(screen.getByRole('button', { name: 'Switch to English' }))
const chinese = () => fireEvent.click(screen.getByRole('button', { name: '切换为中文' }))

it('switches text, attributes and existing notices both ways without remounting inputs', () => {
  render(<LanguageProvider><Example /></LanguageProvider>)
  const input = screen.getByLabelText('物体名')
  fireEvent.change(input, { target: { value: '用户物体' } })
  english()
  expect(screen.getByRole('heading', { name: 'Site map' })).toBeTruthy()
  expect(screen.getByLabelText('Object')).toBe(input)
  expect((input as HTMLInputElement).value).toBe('用户物体')
  expect(input.getAttribute('placeholder')).toBe('Object label')
  expect(screen.getByAltText('Head camera').title).toBe('Live preview')
  expect(screen.getByText('Recorded the position and heading for 地图-机器人-{0}.')).toBeTruthy()
  expect(screen.getByText('/example/地图/机器人.yaml')).toBeTruthy()
  expect(document.documentElement.lang).toBe('en')
  expect(window.localStorage.getItem('xlerobot.language')).toBe('en')
  fireEvent.click(screen.getByText('notice'))
  chinese()
  expect(screen.getByLabelText('物体名')).toBe(input)
  expect(input.getAttribute('placeholder')).toBe('物体标签')
  expect(screen.getByAltText('头部相机').title).toBe('实时预览')
  expect(screen.getByText('已删除 地图-1。')).toBeTruthy()
  expect(document.documentElement.lang).toBe('zh-CN')
})

it('restores the language after reload and tolerates unavailable browser storage', () => {
  window.localStorage.setItem('xlerobot.language', 'en')
  const view = render(<LanguageProvider><Example /></LanguageProvider>)
  expect(screen.getByRole('heading', { name: 'Site map' })).toBeTruthy()
  view.unmount()
  vi.spyOn(Storage.prototype, 'getItem').mockImplementation(() => { throw new Error('blocked') })
  vi.spyOn(Storage.prototype, 'setItem').mockImplementation(() => { throw new Error('blocked') })
  render(<LanguageProvider><Example /></LanguageProvider>)
  expect(screen.getByRole('heading', { name: '场地地图' })).toBeTruthy()
  english()
  expect(screen.getByRole('heading', { name: 'Site map' })).toBeTruthy()
})

it('translates only explicit complete keys and preserves arguments, numbers and placeholders', () => {
  expect(translate('/example/地图/机器人.yaml', 'en')).toBe('/example/地图/机器人.yaml')
  expect(translate('constructor', 'en')).toBe('constructor')
  expect(translateService('toString', 'en')).toBe('toString')
  expect(translate('已删除 {0}。', 'en', '$& {1} 地图')).toBe('Deleted $& {1} 地图.')
  expect(translate(msg('{0}：{1}', '地图', msg('已保留')), 'en')).toBe('地图: Kept')
  expect(translateService('Error: 预览已过期，请重新预览', 'en')).toBe('Error: Preview expired; preview again.')
  expect(translateService('已完成 2/3 个点', 'en')).toBe('Completed 2/3 points.')
  expect(translateService('未知异常 /地图/file.yaml', 'en')).toBe('Original service detail: 未知异常 /地图/file.yaml')
})

it('does not use a document MutationObserver for translation', async () => {
  const observe = vi.spyOn(MutationObserver.prototype, 'observe')
  render(<LanguageProvider><Example /></LanguageProvider>)
  english()
  await act(async () => {})
  expect(observe).not.toHaveBeenCalled()
})
