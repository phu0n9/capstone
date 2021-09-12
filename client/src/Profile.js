import React,{useEffect,useState} from 'react'
import { useAuth0 } from "@auth0/auth0-react"
import CalendarHeatmap from 'react-calendar-heatmap'
import 'react-calendar-heatmap/dist/styles.css'
import axios from 'axios'

const Profile = () => {
  const { user,isAuthenticated,getAccessTokenSilently} = useAuth0()
  const [userDataSearch,setUserDataSearch] = useState([])
  require('dotenv').config()

  useEffect(() =>{
    async function getAccessToken(){
      if(isAuthenticated){
          const token = await getAccessTokenSilently()
          await axios({
            method:'GET',
            url: process.env.REACT_APP_WINDOW_LOCATION+'profile/date', //change here
            params:{userId:user.sub},
            headers:{
                'Authorization':`Bearer ${token}`
            }
        })
          .then(response => {
              setUserDataSearch(prevInventory =>{
                return [...new Set([...prevInventory,...response.data.map(i => i)])]
            })
          })
          .catch(err => console.log(err))
      }
    }
    getAccessToken()
  },[getAccessTokenSilently,isAuthenticated,user])

  return (
    <div className="profile-container">
      <div className="row gutters">
        <div className="col-xl-3 col-lg-3 col-md-12 col-sm-12 col-12">
          <div className="card h-100">
            <div className="card-body">
              <div className="account-settings">
                <div className="user-profile">
                  <div className="user-avatar">
                    <img src={user.picture} alt="profile-pic"/>
                  </div>
                  <h5 className="user-name">{user.nickname}</h5>
                  <h6 className="user-email">{user.email}</h6>
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
                    <input type="text" className="form-control" value={user.name} readOnly/>
                  </div>
                </div>
                <div className="col-xl-6 col-lg-6 col-md-6 col-sm-6 col-12">
                  <div className="form-group">
                    <label htmlFor="eMail">Email</label>
                    <input type="email" className="form-control" value={user.email} readOnly/>
                  </div>
                </div>
              </div>
              <div className="row gutters">
                <div className="col-xl-12 col-lg-12 col-md-12 col-sm-12 col-12">
                  <h6 className="mt-3 mb-2 text-primary">Searches in this year</h6>
                </div>
                  <CalendarHeatmap
                    startDate={new Date('2021-01-01')}
                    endDate={new Date('2022-01-01')}
                    values={
                       userDataSearch
                    }
                 
                    classForValue={(value) => {
                      if (!value) {
                        return 'color-empty';
                      }
                      else if (value.count < 5){
                        return `color-scale-1`
                      }
                      else if (value.count < 10){
                        return `color-scale-2`
                      }
                      else if (value.count <20){
                        return `color-scale-3`
                      }
                      else{
                        return `color-scale-4`
                      }
                    }}
                  />
               
              </div>
              
            </div>
          </div>
        </div>
      </div>
    </div>
    )
}

export default Profile