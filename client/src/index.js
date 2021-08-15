import React from 'react';
import ReactDOM from 'react-dom';
import './stylesheet/style.css';
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

import App from './App';
import {Auth0Provider} from '@auth0/auth0-react'
require('dotenv').config()

ReactDOM.render(
  <React.StrictMode>
    <Auth0Provider
    domain={process.env.REACT_APP_AUTH0_DOMAIN}
    clientId={process.env.REACT_APP_AUTH0_CLIENT_ID}
    redirectUri={window.location.origin}
    audience={process.env.REACT_APP_AUTH0_AUDIENCE}
    scope={process.env.REACT_APP_AUTH0_SCOPE}
    >
      <App />
    </Auth0Provider>
  </React.StrictMode>,
  document.getElementById('root')
);

