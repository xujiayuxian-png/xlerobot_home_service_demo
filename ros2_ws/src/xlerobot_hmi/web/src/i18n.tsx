import { createContext, useContext, useEffect, useMemo, useState, type ReactNode } from 'react'
import { english } from './translations'
import { serviceEnglish } from './serviceTranslations'

export type Language = 'zh' | 'en'
export type Message = string | { source: string, values: readonly (Message | number)[] }
// Store notices as source + arguments, so they switch even after an operation ends.
export const msg = (source: string, ...values: (Message | number)[]): Message => ({ source, values })

export function translate(message: Message | null | undefined, language: Language, ...args: (Message | number)[]): string {
  if (message == null) return ''
  const source = typeof message === 'string' ? message : message.source
  const values = typeof message === 'string' ? args : message.values
  const template = language === 'en' && Object.hasOwn(english, source) ? english[source] : source
  return template.replace(/\{(\d+)\}/g, (placeholder, index: string) => {
    const value = values[Number(index)]
    // Data is inserted verbatim; only explicitly nested messages are translated.
    return value === undefined ? placeholder : typeof value === 'object' ? translate(value, language) : String(value)
  })
}

const storageKey = 'xlerobot.language'
const servicePatterns = Object.entries(serviceEnglish).filter(([source]) => /\{\d+\}/.test(source)).map(([source, target]) => {
  const indices: string[] = []
  const pattern = source.split(/(\{\d+\})/).map(part => {
    if (/^\{\d+\}$/.test(part)) { indices.push(part.slice(1, -1)); return '([\\s\\S]+?)' }
    return part.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')
  }).join('')
  return { pattern: new RegExp(`^${pattern}$`), indices, target }
})

// Only call on service status/error fields, never paths, names, JSON or logs.
export function translateService(message: Message | null | undefined, language: Language): string {
  if (typeof message !== 'string' || language === 'zh') return translate(message, language)
  const prefix = message.startsWith('Error: ') ? 'Error: ' : ''
  const source = prefix ? message.slice(prefix.length) : message
  if (Object.hasOwn(serviceEnglish, source)) return prefix + serviceEnglish[source]
  if (Object.hasOwn(english, source)) return prefix + english[source]
  for (const { pattern, indices, target } of servicePatterns) {
    const match = pattern.exec(source)
    if (match) return prefix + target.replace(/\{(\d+)\}/g, (_, index: string) => match[indices.indexOf(index) + 1])
  }
  // Keep unexpected diagnostics intact, with an English explanation of the raw text.
  return /\p{Script=Han}/u.test(source) ? `Original service detail: ${message}` : message
}

type LanguageContextValue = {
  language: Language, setLanguage: (value: Language) => void,
  t: (message: Message | null | undefined, ...values: (Message | number)[]) => string,
  s: (message: Message | null | undefined) => string,
}
const LanguageContext = createContext<LanguageContextValue>({
  language: 'zh', setLanguage: () => undefined, t: (message, ...values) => translate(message, 'zh', ...values),
  s: message => translateService(message, 'zh'),
})

export function LanguageProvider({ children }: { children: ReactNode }) {
  const [language, setLanguageState] = useState<Language>(() => {
    try { return window.localStorage.getItem(storageKey) === 'en' ? 'en' : 'zh' }
    catch { return 'zh' }
  })
  const context = useMemo<LanguageContextValue>(() => ({
    language,
    setLanguage(value) {
      setLanguageState(value)
      try { window.localStorage.setItem(storageKey, value) } catch { /* Storage is optional. */ }
    },
    t: (message, ...values) => translate(message, language, ...values),
    s: message => translateService(message, language),
  }), [language])
  useEffect(() => {
    document.documentElement.lang = language === 'zh' ? 'zh-CN' : 'en'
    if (document.querySelector('meta[name="xlerobot-workbench"]')) {
      document.title = language === 'zh' ? 'XLeRobot · 机器人标定' : 'XLeRobot · Robot calibration'
    }
  }, [language])
  return <LanguageContext.Provider value={context}>{children}</LanguageContext.Provider>
}

export function useLanguage() { return useContext(LanguageContext) }

export function LanguageToggle() {
  const { language, setLanguage } = useLanguage()
  return <button type="button" className="language-toggle" lang={language === 'zh' ? 'en' : 'zh-CN'}
    aria-label={language === 'zh' ? 'Switch to English' : '切换为中文'}
    onClick={() => setLanguage(language === 'zh' ? 'en' : 'zh')}>
    {language === 'zh' ? 'EN' : '中文'}
  </button>
}
