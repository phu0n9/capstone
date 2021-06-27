import React,{useCallback,useState,useRef,useEffect} from 'react'
import infinityScroll from './InfinityScroll'
import Calendar from 'react-calendar'
import 'react-calendar/dist/Calendar.css'

export default function SidebarBox({setClickPhoto}) {
    const observer = useRef()
    const [pageNumber,setPageNumber] = useState(5)
    const [keyword,setKeyword] = useState()
    const [keypress,setKeypress] = useState(false)
    const [selection,setSelection] = useState("location")
    // const [value, setCalendar] = useState(new Date())
    // const [type,setType] = useState("search")
    // const [pType,setPType] = useState("hidden")
    
    const {
        inventory,
        hasMore,
        error,
        loading
    } = infinityScroll(pageNumber,keyword,selection)
    

    const lastInventory= useCallback((node) =>{
        if(loading) return
        if(observer.current) observer.current.disconnect()
        observer.current = new IntersectionObserver(entries =>{
            if(entries[0].isIntersecting && hasMore)
            setPageNumber(prevInventory => prevInventory + 5)
        })
        if(node) observer.current.observe(node)
    },[loading,hasMore])

    const handleItemSearch = ((e) =>{
        setKeyword(e.target.value)
    })

    const handleKeyPress = ((e) =>{
        if(e.key === "Enter"){
            setKeypress(true)
            console.log("keyword: "+  keyword)
        }
    })

    const handleSelection = ((e) =>{
        setSelection(e.target.value)
    })

    useEffect(() =>{
        if(keypress === true){
            if(keyword === undefined)  {
                alert("Please enter a keyword")
            }
            else{
                console.log("sending to back end")
                if(selection === "userId"){
                    setKeyword(localStorage.getItem('userId'))
                }
                // setKeypress(false)
                // setKeyword("")
            }
        }
    },[keyword,keypress,selection])

    // useEffect(()=>{
    //     if(selection ==="date"){
    //         console.log("this")
    //         setType("hidden")
    //         setKeyword(value)
    //         setPType("text")
    //     }
    //     else{
    //         setType("search")
    //         setPType("hidden")
    //     }
    // },[selection,value])


    return ( 
        <div className="grid-item sidebar-wrapper">
            <div className="search-item-bar">
                <input
                    type="search"
                    placeholder="Search existing item"
                    className="search-item"
                    onChange={handleItemSearch}
                    onKeyUp={handleKeyPress}
                    value ={keyword}
                />
                 {/* <Calendar
                    className="calendar"
                    onChange={setCalendar}
                    value={value}
                /> */}
            <p className="sort-title">Sort by:</p>
            <select name="sort" className="sort-select" onChange={handleSelection} value={selection}>
                <option value="location" selected>Location</option>
                <option value="userId">Create By You</option>
                <option value="date">Date</option>
            </select>
            </div>
            {inventory.map((item,index)=>{
                if(inventory.length === index +1){
                    return <div className="inventory-item">
                        <div key={item.location} ref={lastInventory}>Location: {item.location}</div>
                        <img key={item.photo} ref={lastInventory} src={item.photo} alt="sideBarPhoto" onClick={() => setClickPhoto(item.photo)} className="img-sideBar"/>
                    </div>
                }
                else{
                    return <div className="inventory-item">
                        <div key={item.location}>Location: {item.location}</div>
                        <img key={item.photo} src={item.photo} alt="sideBarPhoto" onClick={() => setClickPhoto(item.photo)} className="img-sideBar"/>
                </div>
                }
            })}
            <div>{loading && 'Loading...'}</div>
            <div>{error && 'Error'}</div>
        </div>
    )
}
