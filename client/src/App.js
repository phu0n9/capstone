import Homepage from './Homepage'
import "bootstrap/dist/css/bootstrap.min.css";
import {
  BrowserRouter as Router,
  Route,
  Switch
} from 'react-router-dom'
import React from 'react'
import ProtectedRoute from './Route/ProtectedRoute'
import ProfileMain from './ProfileMain'

function App() {
  return (
  <Router>
      <Switch>
        <ProtectedRoute component={Homepage} path="/" exact />
        <ProtectedRoute component={ProfileMain} path="/profile" exact />
        {/* <Route component={Homepage} path="/" exact /> */}
        <Route path="*" component={() => "404 NOT FOUND"}/>
      </Switch>
    </Router>
  )
}

export default App

