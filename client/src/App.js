import Homepage from './Homepage'
import Login from './Login'
import "bootstrap/dist/css/bootstrap.min.css";
import {
  BrowserRouter as Router,
  Switch,
  Route,
  Redirect
} from 'react-router-dom'
import React from 'react'

function App() {
  return (
  <Router>
    <Switch>
      <Route path="/" exact>
        <Redirect to="/login"/>
      </Route>

      <Route path="/login" component={Login}/>

      <Route path="/homepage" component={Homepage}/>
    </Switch>
  </Router>   
  );
}

export default App;

