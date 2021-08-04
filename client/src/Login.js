import React,{useState} from 'react'
import axios from 'axios'

export default function Login() {
    const [username, setUserName] = useState()
    const [password, setPassword] = useState()
    const heroku = 'https://schaeffler.herokuapp.com/login'
    // 'http://localhost:5000/login'
    const handleSubmit = async e => {
        e.preventDefault();
        const content = {
          username:username,
          password:password
        }
        await axios.post('http://localhost:5000/login',content)
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
          <div>    </div>
        <form onSubmit={handleSubmit} method="post" action="/login" className="form-wrapper">
        <img src="schaeffler-logo.jpg" className="login-logo" alt="login-logo"/><br></br>
            <input type="text" placeholder="username" className="input-style" onChange={e => setUserName(e.target.value)} required/>
          <br></br>
            <input type="password" placeholder="password" className="input-style" onChange={e => setPassword(e.target.value)} required/>
          <div>
            <button type="submit" className="login-btn">LOGIN</button>
          </div>
        </form>
    </div>
    )
}