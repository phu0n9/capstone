import React,{useCallback,useState,useRef,useEffect} from 'react'
import infinityScroll from './InfinityScroll'
import Calendar from 'react-calendar'
import 'react-calendar/dist/Calendar.css'

export default function SidebarBox({setClickPhoto}) {
    const observer = useRef()
    const [pageNumber,setPageNumber] = useState(5)
    const [keyword,setKeyword] = useState('')
    const [selection,setSelection] = useState("location")
    const [enableCalendar,setEnableCalendar] = useState(false)
    const [value, setCalendar] = useState(new Date())
    const [buttonType,setButtonType] = useState(false)
    const [keydown,setKeyDown] = useState("")
    const [state,setState] = useState(false)
    
    const {
        inventory,
        hasMore,
        error,
        loading,change
    } = infinityScroll(pageNumber,keyword,selection)
    

    const lastInventory= useCallback((node) =>{
        if(loading) return
        if(observer.current) observer.current.disconnect()
        observer.current = new IntersectionObserver(entries =>{
            if(entries[0].isIntersecting && hasMore) setPageNumber(prevInventory => prevInventory + 5)
        })
        if(node) observer.current.observe(node)
    },[loading,hasMore])

    const handleItemSearch = ((e) =>{
        setKeyword(e.target.value)
    })

    const handleSelection = ((e) =>{
        setSelection(e.target.value)
    })

    useEffect(() =>{
        if (change){
            setKeyword("")
        }
    },[change])

    useEffect(()=>{
        
        switch(selection){
            case "date":
                setEnableCalendar(true)
                setKeyword(value.getFullYear()+'-' + (value.getMonth()+1) + '-'+value.getDate())
                setButtonType(true) 
                setState(false)
                break
            case "userId":
                setEnableCalendar(false)
                setButtonType(true)
                if (keyword !== undefined) {
                    setKeyword(localStorage.getItem('userId'))
                }
                setState(false)
                break
            default:
                if(keyword === localStorage.getItem('userId') || keyword === value.getFullYear()+'-' + (value.getMonth()+1) + '-'+value.getDate()) {
                    setKeyword("")
                }
                setEnableCalendar(false)
                setButtonType(false)
                setState(true)
                break
        }
    },[selection,value,keyword])

    useEffect(() =>{
        if (keydown !== "" && keyword === ""){
            setPageNumber(5)
        }
    },[keydown,keyword])

    const handleKeyDown = ((e) =>{
        setKeyDown(e.target.value)
    })

    return ( 
        <div className="grid-item sidebar-wrapper">
            <div className="search-item-bar">
                {enableCalendar ?
                 <Calendar
                    className="calendar"
                    onChange={setCalendar}
                    value={value}
                    next2Label=""
                    prev2Label=""
                /> : 
                <input
                    type={buttonType ? "hidden": "search"}
                    placeholder="Search existing item"
                    className="search-item"
                    onChange={handleItemSearch}
                    value ={keyword}
                    disabled={buttonType ? "disabled": ""}
                    onKeyDown={handleKeyDown}
                />}
            <p className="sort-title">Sort by:</p>
            <select name="sort" className="sort-select" onChange={handleSelection} value={selection}>
                <option value="location" defaultValue>Location</option>
                <option value="userId">Create By You</option>
                <option value="date">Date</option>
            </select>
            </div>
            {inventory.map((item,index)=>{
                if(inventory.length === index +1){
                    return <div className="inventory-item" key={index.toString()}>
                        <div key={(index+1).toString()} ref={lastInventory}>Location: {item.location}</div>
                        <div key={(index+2).toString()}>Create At: {item.createdAt.substring(0,10)}</div>
                        <img key={item.photo} ref={lastInventory} src={`data:image/png;base64,${item.photo}`} alt="sideBarPhoto" onClick={() => setClickPhoto(item.photo)} className="img-sideBar"/>
                    </div>
                }
                else{
                    return <div className="inventory-item" key={index.toString()}>
                        <div key={(index+1).toString()}>Location: {item.location}</div>
                        <div key={(index+2).toString()}>Create At: {item.createdAt.substring(0,10)}</div>
                        <img key={item.photo} src={`data:image/png;base64,${item.photo}`} alt="sideBarPhoto" onClick={() => setClickPhoto(item.photo)} className="img-sideBar"/>
                    </div>
                }
            })}
            {state ? <div>{loading && 'Loading...'}</div> : ""}
            <div>{error && 'Error'}</div>
        </div>
    )
}
