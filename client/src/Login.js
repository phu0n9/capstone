import React,{useState} from 'react'
import axios from 'axios'

export default function Login() {
    const [username, setUserName] = useState()
    const [password, setPassword] = useState()

    const handleSubmit = async e => {
        e.preventDefault();
        const content = {
          username:username,
          password:password
        }
        await axios.post('https://schaeffler.herokuapp.com/login',content)
        .then(data =>{
          if(data.data === 'Incorrect password' || data.data === 'Cannot find user'){
            alert(data.data)
          }
          else{
            localStorage.setItem("userId",data.data)
            window.location = "/homepage"
          }
        })
        .catch(error => console.log(error))
    }

    return (
        <div className="login-wrapper">
      <h1>Logo</h1>
      <form onSubmit={handleSubmit} method="post" action="/login">
        <label>
          <p>Username</p>
          <input type="text" onChange={e => setUserName(e.target.value)} required/>
        </label>
        <br></br>
        <label>
          <p>Password</p>
          <input type="password" onChange={e => setPassword(e.target.value)} required/>
        </label>
        <div>
          <button type="submit">Submit</button>
        </div>
      </form>
    </div>
    )
}