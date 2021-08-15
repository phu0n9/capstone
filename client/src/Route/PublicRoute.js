import React from 'react'
import { Route, Redirect } from 'react-router-dom'

const PublicRoute = ({component: Component,restricted,...rest}) => {
    return (
        <Route {...rest} render={props => (
            restricted ? 
            <Redirect to="/homepage" />
            :  
            <Component {...props} />
        )} />
    )
}

export default PublicRoute