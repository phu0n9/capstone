import React,{useCallback,useState,useRef,useEffect} from 'react'
import infinityScroll from './InfinityScroll'

export default function SidebarBox({setClickPhoto}) {
    const observer = useRef()
    const [pageNumber,setPageNumber] = useState(5)
    
    const {
        inventory,
        hasMore,
        error,
        loading,
        change
    } = infinityScroll(pageNumber)
    

    const lastInventory= useCallback((node) =>{
        if(loading) return
        if(observer.current) observer.current.disconnect()
        observer.current = new IntersectionObserver(entries =>{
            if(entries[0].isIntersecting && hasMore)
            setPageNumber(prevInventory => prevInventory + 5)
        })
        if(node) observer.current.observe(node)
    },[loading,hasMore])

    // useEffect(() => {
    //     if(change){
    //         setPageNumber(5)
    //     }
    // },[change])

    return ( <div className="grid-item sidebar-wrapper">
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
