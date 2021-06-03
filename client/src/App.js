import Homepage from './Homepage'
import "bootstrap/dist/css/bootstrap.min.css";
import {
  BrowserRouter as Router,
  Switch,
  Route,
  Redirect
} from 'react-router-dom'
import {v4 as uuidV4} from 'uuid'
import React from 'react'

function App() {
  return (
  <Router>
    <Switch>
      <Route path="/" exact>
        <Redirect to={`/user=${uuidV4()}`}/>
      </Route>
      <Route path="/user=:id" component={Homepage}>
        <Homepage/>
      </Route>
    </Switch>
  </Router>   
  );
}

export default App;

