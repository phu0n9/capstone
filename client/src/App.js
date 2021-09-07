import Homepage from './Homepage'
import "bootstrap/dist/css/bootstrap.min.css";
import {
  BrowserRouter as Router,
  Route,
  Switch
} from 'react-router-dom'
import React from 'react'
import ProtectedRoute from './Route/ProtectedRoute'

function App() {
  return (
  <Router>
      <Switch>
        <ProtectedRoute component={Homepage} path="/" exact />
        <ProtectedRoute component={Homepage} path="/profile" exact />
        {/* <Route path="/" component={Homepage} exact/> */}
        <Route path="*" component={() => "404 NOT FOUND"}/>
      </Switch>
    </Router>
  )
}

export default App

