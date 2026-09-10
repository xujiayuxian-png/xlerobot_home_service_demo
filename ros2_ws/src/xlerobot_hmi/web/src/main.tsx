import { StrictMode } from 'react'
import { createRoot } from 'react-dom/client'
import { App } from './App'
import { CalibrationWorkbench } from './CalibrationWorkbench'
import { LanguageProvider } from './i18n'
import './style.css'

createRoot(document.getElementById('root')!).render(
  <StrictMode><LanguageProvider>{document.querySelector('meta[name="xlerobot-workbench"]')
    ? <CalibrationWorkbench /> : <App />}</LanguageProvider></StrictMode>,
)
