/// <reference types="vite/client" />
import ts from 'typescript'
import { expect, it } from 'vitest'
import { english } from './translations'
import { serviceEnglish } from './serviceTranslations'

it('covers every authored Chinese UI message and preserves placeholder signatures', () => {
  const data = new Set(['羽毛球', '抓住羽毛球']) // Dataset defaults, not UI labels.
  const missing: string[] = []
  const sources = import.meta.glob<string>(['./*.tsx', './*.ts', '!./*.test.*', '!./i18n.tsx', '!./translations.ts', '!./serviceTranslations.ts'],
    { query: '?raw', import: 'default', eager: true })
  for (const [filename, content] of Object.entries(sources)) {
    const source = ts.createSourceFile(filename, content, ts.ScriptTarget.Latest, true)
    function visit(node: ts.Node) {
      if (ts.isStringLiteral(node) && /\p{Script=Han}/u.test(node.text)
        && !data.has(node.text) && !(node.text in english)) missing.push(`${filename}: ${node.text}`)
      if (ts.isJsxText(node) && /\p{Script=Han}/u.test(node.text)) missing.push(`${filename}: untranslated JSX ${node.text}`)
      if (ts.isTemplateExpression(node) && /\p{Script=Han}/u.test(node.head.text + node.templateSpans.map(span => span.literal.text).join(''))
        && !(ts.isCallExpression(node.parent) && node.parent.expression.getText(source) === 'setInstruction')) {
        missing.push(`${filename}: untranslated template ${node.getText(source)}`)
      }
      ts.forEachChild(node, visit)
    }
    visit(source)
  }
  expect(missing).toEqual([])
  const placeholders = (value: string) => [...value.matchAll(/\{\d+\}/g)].map(match => match[0]).sort()
  for (const [source, target] of Object.entries({ ...english, ...serviceEnglish })) {
    expect(target, source).not.toMatch(/\p{Script=Han}/u)
    expect(placeholders(target), source).toEqual(placeholders(source))
  }
})
