import { StrictMode } from 'react'
import { createRoot } from 'react-dom/client'
import { App } from './App'
import { CalibrationWorkbench } from './CalibrationWorkbench'
import './style.css'

createRoot(document.getElementById('root')!).render(
  <StrictMode>{document.querySelector('meta[name="xlerobot-workbench"]')
    ? <CalibrationWorkbench /> : <App />}</StrictMode>,
)
