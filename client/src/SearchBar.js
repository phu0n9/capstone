import React,{useEffect,useState} from 'react'
import {io} from 'socket.io-client'

export default function SearchBar({userId}) {
  
    const [socket,setSocket] = useState()
    const [clicked,setClick] = useState(false)
    const [keyword,setKeyword] = useState()
    const heroku = 'https://schaeffler.herokuapp.com/'

    // 'http://localhost:5000/'
    useEffect(() => {
        const s = io(heroku)
        setSocket(s)
        return () =>{
            s.disconnect()  
        }
    }, []) 

    const onSearchClickButton = () =>
    {
        setClick(!clicked)
    }

    const onSearching = (event) =>{
        setKeyword(event.target.value);
    }

    useEffect(() =>{
        if (socket === null || keyword === null) return
        if(clicked === true && keyword !== null){
            const key = {
                "keyword":keyword,
                "userId":userId
            }
            socket.emit("begin-search",key)
            setClick(!clicked)
            setKeyword("")
        }
    },[socket,keyword,clicked,userId])

    return (
        <span className="search-bar">
        <form>
            <input
                type="search"
                placeholder="Search inventory"
                className="search-box"
                onChange={onSearching}
                value={keyword}
            />
            <button type="button" className="search-btn" onClick={onSearchClickButton}>Search</button>
        </form>
        </span>
    )
}
