import {useEffect,useState} from 'react'
import axios from 'axios'
import Pusher from 'pusher-js'

export default function InfinityScroll(pageNumber,keyword,selection) {
    const [loading,setLoading] = useState(true)
    const [error,setError] = useState(false)
    const [inventory,setInventory] = useState([])
    const [hasMore,setHasMore] = useState(false)       
    const heroku = 'https://schaeffler.herokuapp.com/inventory'
    // 'http://localhost:5000/inventory'
    useEffect(() =>{
        const pusher = new Pusher('2ccb32686bdc0f96f50a',{
            'cluster':'ap1',
            encrypted:true
        })
        const channel = pusher.subscribe('tasks')
        channel.bind('inserted',function(){
            // if(keyword != null){

            // }
            // else {
                axios({
                    method:'GET',
                    url: heroku,
                    params:{page:5}
                })
                .then(res => {
                    setInventory(() =>{
                        return [...new Set([...'',...res.data.map(i => i)])]
                    })
                })
                .catch(e =>{
                    setError(true)
                    console.log('Error: '+e)
                })
            // } 
        })
        return () => channel.unbind('inserted')
    },[keyword])


    function getApi(URL,paramsDict){
        axios({
            method:'GET',
            url: URL,
            params:paramsDict
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

    const refresh = () =>{
        setLoading(true)
        setError(false)
        if(keyword != null){
            if (selection === "location"){
                axios({
                    method:'GET',
                    url: 'http://localhost:5000/sort',
                    params:{location:keyword}
                })
                .then(res => {
                    setInventory(() =>{
                        return [...new Set([...'',...res.data.map(i => i)])]
                    })
                    setHasMore(res.data.length > 0)
                    setLoading(false)
                })
                .catch(e =>{
                    setError(true)
                    console.log('Error: '+e)
                })
            }
        }
        else{
            getApi('http://localhost:5000/inventory',{page:pageNumber})
        }
    }

    useEffect((refresh),[pageNumber,keyword,selection])

    return {loading,hasMore,error,inventory}
}
