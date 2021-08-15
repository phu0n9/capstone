import Homepage from './Homepage'
import HandleLogin from './HandleLogin'
import "bootstrap/dist/css/bootstrap.min.css";
import {
  BrowserRouter as Router,
  Switch
} from 'react-router-dom'
import React from 'react'
import {useAuth0} from '@auth0/auth0-react'
import PublicRoute from './Route/PublicRoute'
import PrivateRoute from './Route/PrivateRoute'

function App() {
  const {isAuthenticated} = useAuth0()
  return (
  <Router>
      <Switch>
        <PublicRoute restricted={isAuthenticated} component={HandleLogin} path="/" exact/>
        <PrivateRoute restricted={isAuthenticated} component={Homepage} path="/homepage" exact />
      </Switch>
    </Router>
  )
}

export default App

