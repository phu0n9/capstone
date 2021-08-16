import React from 'react'
import ReactDOM from 'react-dom'
import './stylesheet/style.css'
import './stylesheet/batteryStyle.css'
import './stylesheet/login.css'
import './stylesheet/searchBar.css'
import './stylesheet/sideBarBox.css'
import './stylesheet/mapping.css'
import './stylesheet/photoDisplay.css'
import './stylesheet/carStatus.css'
import './stylesheet/droneStatus.css'
import './stylesheet/setting.css'
import './stylesheet/darkMode.css'
import './stylesheet/pop-up.css'

import App from './App'

import { BrowserRouter as Router } from "react-router-dom"
import Auth0ProviderWithHistory from "./auth0-provider-with-history"

ReactDOM.render(
  <Router>
    <Auth0ProviderWithHistory>
      <App />
    </Auth0ProviderWithHistory>
  </Router>,
  document.getElementById("root")
)
