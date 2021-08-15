import React from 'react'
import { Route, Redirect } from 'react-router-dom'

const PrivateRoute = ({component: Component,restricted, ...rest}) => {
    return (
        <Route {...rest} render={props => (
            restricted ?
            <Component {...props} />
            :            
            <Redirect to="/" />
        )} />
    )
}

export default PrivateRoute