import React from 'react'
import { useAuth0 } from "@auth0/auth0-react"

const Profile = () => {
  const { user, isAuthenticated } = useAuth0()

  return (
      <>
        Profile page
      </>
    )
}

export default Profile