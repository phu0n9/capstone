import {useEffect,useState} from 'react'
import axios from 'axios'
import Pusher from 'pusher-js'

export default function InfinityScroll(pageNumber,keyword,selection) {
    const [loading,setLoading] = useState(true)
    const [error,setError] = useState(false)
    const [inventory,setInventory] = useState([])
    const [hasMore,setHasMore] = useState(false)       
    const heroku = 'https://schaeffler.herokuapp.com/inventory'

    function fetchApi(URL,paramsDict){
        axios({
            method:'GET',
            url: URL,
            params:paramsDict
        })
        .then(res => {
            setInventory(() =>{
                return [...new Set([...'',...res.data.map(i => i)])]
            })
            // setHasMore(res.data.length > 0)
            // setLoading(false)
        })
        .catch(e =>{
            setError(true)
            console.log('Error: '+e)
        })
    }
    
    // 'http://localhost:5000/inventory'
    useEffect(() =>{
        const pusher = new Pusher('2ccb32686bdc0f96f50a',{
            'cluster':'ap1',
            encrypted:true
        })
        const channel = pusher.subscribe('tasks')
        channel.bind('inserted',function(){
            fetchApi(heroku,{page:5})
        })
        return () => channel.unbind('inserted')
    },[keyword])


    useEffect(() =>{
        if(keyword !== undefined){
            switch(selection){
                case "location":
                    fetchApi('https://schaeffler.herokuapp.com/sort/sortByLocation',{location:keyword})
                    break
                case "userId":
                    fetchApi('https://schaeffler.herokuapp.com/sort/sortByUserId',{userId:keyword})
                    break
                case "date":
                    fetchApi('https://schaeffler.herokuapp.com/sort/sortByTime',{time:keyword})
                    break
                default:
                    break
            }
        }
    },[keyword,selection])

    useEffect(() =>{
        setLoading(true)
        setError(false)
        if(keyword === ''){
            axios({
                method:'GET',
                url: heroku,
                params:{page:pageNumber}
            })
            .then(res => {
                setInventory(prevInventory =>{
                    return [...new Set([...prevInventory,...res.data.map(i => i)])]
                })
                    setHasMore(res.data.length > 0)
                    setLoading(false)
            })
            .catch(e =>{
                setError(true)
                console.log('Error: '+e)
            })
        }
    },[pageNumber,keyword])

    return {loading,hasMore,error,inventory}
}
