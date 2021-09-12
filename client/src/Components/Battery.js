import React,{useState,useEffect} from 'react'

export default function Battery({value}) {
    const [color, setColor] = useState("green")

    useEffect(()=>{
        if(value<=30){
            setColor("red")
        }else if(value<=70){
            setColor("orange")
        }else setColor("green")
    },[value])
    
      return(
        <div className="d-flex align-items-center float-right">
            <div className="fa-battery-filling">
                <span className={color} style={{width:`calc(${value}% * 0.95)`}}>
                    <h1 className="battery-level">{value}%</h1>
                </span>
            </div>
            <div className="battery-head"></div>
        </div>
        
      )
}
