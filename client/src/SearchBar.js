import React,{useEffect,useState} from 'react'
import {io} from 'socket.io-client'

export default function SearchBar({userId}) {
  
    const [socket,setSocket] = useState()
    const [pressed,setPress] = useState(false)
    const [keyword,setKeyword] = useState()
    const [buttonClicked,setButtonClicked] = useState(false)
    // const [clicked,setClicked] = useState(false)
    const heroku = 'https://schaeffler.herokuapp.com/'

    // 'http://localhost:5000/'
    useEffect(() => {
        const s = io(heroku)
        setSocket(s)
        return () =>{
            s.disconnect()  
        }
    }, []) 

    const onPress = (event) =>
    {
        if(event.key === 'Enter'){
            setPress(true)
        }
    }

    const onSearchClickButton = () =>
    {
        // setClicked(true)
        setButtonClicked(true)
    }

    const onSearching = (event) =>{
        setKeyword(event.target.value);
    }

    useEffect(() =>{
        if (socket === null || keyword === null) return

        if((pressed === true ) || (buttonClicked === true)){
            if (keyword !== ""){
                const key = {
                    "keyword":keyword,
                    "userId":userId
                }
                socket.emit("begin-search",key)
                setPress(false)
                setButtonClicked(false)
                setKeyword("")
            }
            else{
                alert("Please enter a keyword")
            } 
        }
    },[socket,keyword,pressed,userId,buttonClicked])

    // const onInputClick = (()=>{
    //     setClicked(true)
    // })

    return (
        <span >
            <input
                type="search"
                placeholder="Search inventory"
                className="search-box"
                onChange={onSearching}
                value={keyword}
                onKeyPress={onPress}
                // onClick={onInputClick}
                // style = {clicked ? {width:450} : {width:300}}
            />
            <button type="button" className="search-btn" onClick={onSearchClickButton}>          
                <img src="search.png" alt="manifying-glass" />
            </button>
        
        </span>
    )
}
