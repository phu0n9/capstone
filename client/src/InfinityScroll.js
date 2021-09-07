import {useEffect,useState} from 'react'
import axios from 'axios'
import Pusher from 'pusher-js'
import {useAuth0} from '@auth0/auth0-react'
require('dotenv').config()

export default function InfinityScroll(pageNumber,keyword) {
    const [loading,setLoading] = useState(true)
    const [error,setError] = useState(false)
    const [inventory,setInventory] = useState([])
    const [hasMore,setHasMore] = useState(false)       
    const heroku = 'https://schaeffler.herokuapp.com/'
    const {isAuthenticated,getAccessTokenSilently} = useAuth0()

   
    // 'http://localhost:5000/inventory'
    useEffect(() =>{
        async function fetchApi(URL,paramsDict){
            if(isAuthenticated){
                const token = await getAccessTokenSilently()
                await axios({
                    method:'GET',
                    url: URL,
                    params:paramsDict,
                    headers: {
                        authorization:`Bearer ${token}`
                    }
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
            }
        }    
        const pusher = new Pusher(process.env.REACT_APP_PUSHER_KEY,{
            'cluster':process.env.REACT_APP_PUSHER_CLUSTER,
            forceTLS: true,
        })
        const channel = pusher.subscribe('tasks')
        channel.bind('inserted',function(){
            // fetchApi('http://localhost:5000/inventory',{page:5}) //change here
            fetchApi(heroku+'inventory',{page:5}) //change here
        })
        return () => channel.unbind('inserted')
    },[isAuthenticated,getAccessTokenSilently])

    useEffect(() =>{
        async function getPages(){
            if(isAuthenticated){
                const token = await getAccessTokenSilently()
                setLoading(true)
                setError(false)
                await axios({
                    method:'GET',
                    // url: 'http://localhost:5000/inventory', //change here
                    url: heroku+'inventory', //change here
                    params:{page:pageNumber},
                    headers:{
                        authorization:`Bearer ${token}`
                    }
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
        }
        
        getPages()
    },[pageNumber,isAuthenticated,getAccessTokenSilently])

    return {loading,hasMore,error,inventory}
}
