import { StrictMode } from 'react'
import { createRoot } from 'react-dom/client'
import './index.css'
import InvestorDemo from './InvestorDemo.jsx'

createRoot(document.getElementById('root')).render(
  <StrictMode>
    <InvestorDemo />
  </StrictMode>,
)
