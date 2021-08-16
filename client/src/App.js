import Homepage from './Homepage'
import "bootstrap/dist/css/bootstrap.min.css";
import {
  BrowserRouter as Router,
  Route,
  Switch
} from 'react-router-dom'
import React from 'react'
import {useAuth0} from '@auth0/auth0-react'
import ProtectedRoute from './Route/ProtectedRoute'

function App() {
  const {isAuthenticated} = useAuth0()
  return (
  <Router>
      <Switch>
        <ProtectedRoute restricted={isAuthenticated} component={Homepage} path="/" exact />
        <Route path="*" component={() => "404 NOT FOUND"}/>
      </Switch>
    </Router>
  )
}

export default App

