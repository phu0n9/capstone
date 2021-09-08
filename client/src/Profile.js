import React,{useEffect,useState} from 'react'
import { useAuth0 } from "@auth0/auth0-react"
import axios from 'axios'

const Profile = () => {
  const { user, isAuthenticated,getAccessTokenSilently } = useAuth0()

  useEffect(() =>{
    async function getAccessToken(){
        if(isAuthenticated){
            const token = await getAccessTokenSilently()
            await axios.get('http://localhost:5000/protected',{//change here
            // await axios.get(heroku+'protected',{//change here
                headers: {
                    authorization:`Bearer ${token}`
                }
            })
            .then(response => {
              console.log(response.data)
            })
            .catch(err => console.log(err))
        }
    }
    getAccessToken()  
  },[getAccessTokenSilently,isAuthenticated])

  return (
    <div className="profile-container">
      <div className="row gutters">
        <div className="col-xl-3 col-lg-3 col-md-12 col-sm-12 col-12">
          <div className="card h-100">
            <div className="card-body">
              <div className="account-settings">
                <div className="user-profile">
                  <div className="user-avatar">
                    <img src="https://bootdey.com/img/Content/avatar/avatar7.png" alt="profile-pic"/>
                  </div>
                  <h5 className="user-name">Yuki Hayashi</h5>
                  <h6 className="user-email">yuki@Maxwell.com</h6>
                </div>
                <div className="about">
                  <h5>About</h5>
                  <p>I'm Yuki. Full Stack Designer I enjoy creating user-centric, delightful and human experiences.</p>
                </div>
              </div>
            </div>
          </div>
          </div>
          <div className="col-xl-9 col-lg-9 col-md-12 col-sm-12 col-12">
          <div className="card h-100">
            <div className="card-body">
              <div className="row gutters">
                <div className="col-xl-12 col-lg-12 col-md-12 col-sm-12 col-12">
                  <h6 className="mb-2 text-primary">Personal Details</h6>
                </div>
                <div className="col-xl-6 col-lg-6 col-md-6 col-sm-6 col-12">
                  <div className="form-group">
                    <label htmlFor="fullName">Full Name</label>
                    <input type="text" className="form-control" placeholder="Enter full name"/>
                  </div>
                </div>
                <div className="col-xl-6 col-lg-6 col-md-6 col-sm-6 col-12">
                  <div className="form-group">
                    <label htmlFor="eMail">Email</label>
                    <input type="email" className="form-control" id="eMail" placeholder="Enter email ID"/>
                  </div>
                </div>
                <div className="col-xl-6 col-lg-6 col-md-6 col-sm-6 col-12">
                  <div className="form-group">
                    <label htmlFor="phone">Phone</label>
                    <input type="text" className="form-control" placeholder="Enter phone number"/>
                  </div>
                </div>
                <div className="col-xl-6 col-lg-6 col-md-6 col-sm-6 col-12">
                  <div className="form-group">
                    <label htmlFor="website">Website URL</label>
                    <input type="url" className="form-control" id="website" placeholder="Website url"/>
                  </div>
                </div>
              </div>
              <div className="row gutters">
                <div className="col-xl-12 col-lg-12 col-md-12 col-sm-12 col-12">
                  <h6 className="mt-3 mb-2 text-primary">Address</h6>
                </div>
                <div className="col-xl-6 col-lg-6 col-md-6 col-sm-6 col-12">
                  <div className="form-group">
                    <label htmlFor="Street">Street</label>
                    <input type="name" className="form-control" placeholder="Enter Street"/>
                  </div>
                </div>
                <div className="col-xl-6 col-lg-6 col-md-6 col-sm-6 col-12">
                  <div className="form-group">
                    <label htmlFor="ciTy">City</label>
                    <input type="name" className="form-control" placeholder="Enter City"/>
                  </div>
                </div>
                <div className="col-xl-6 col-lg-6 col-md-6 col-sm-6 col-12">
                  <div className="form-group">
                    <label htmlFor="sTate">State</label>
                    <input type="text" class="form-control" placeholder="Enter State"/>
                  </div>
                </div>
                <div className="col-xl-6 col-lg-6 col-md-6 col-sm-6 col-12">
                  <div className="form-group">
                    <label htmlFor="zIp">Zip Code</label>
                    <input type="text" className="form-control" placeholder="Zip Code"/>
                  </div>
                </div>
              </div>
              <div className="row gutters">
                <div className="col-xl-12 col-lg-12 col-md-12 col-sm-12 col-12">
                  <div className="text-right">
                    <button type="button" name="submit" className="btn btn-secondary">Cancel</button>
                    <button type="button" name="submit" className="btn btn-primary" style={{backgroundColor: '#358858',borderColor:'#358858'}}>Update</button>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
    )
}

export default Profile