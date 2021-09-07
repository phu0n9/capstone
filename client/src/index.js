import React from 'react'
import ReactDOM from 'react-dom'
import './stylesheet/style.css'
import './stylesheet/login.css'
import './stylesheet/searchBar.css'
import './stylesheet/pictureGallery.css'
import './stylesheet/mapping.css'
import './stylesheet/photoDisplay.css'
import './stylesheet/controlBox.css'
import './stylesheet/deviceStatus.css'
import './stylesheet/queue.css'

import './stylesheet/components/battery.css'
import './stylesheet/components/header.css'
import './stylesheet/components/sidebar.css'
import 'intro.js/introjs.css'

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
