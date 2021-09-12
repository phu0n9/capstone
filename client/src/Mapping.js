import React,{useRef,useEffect} from 'react'

export default function Mapping({rowDes,rackDes,available,setRackClick,setRowClick}) {

      const canvas = useRef()
      const imgRef = useRef()
      const droneRef = useRef()
      const canvasWidth = 1000
      const canvasHeight = 900
      const droneX = 900
      const droneY = 500
      const margin = 50

    const draw = (context) => {
        context.drawImage(imgRef.current, 10, 10,imgRef.current.width, imgRef.current.height)
    }
    
    const handleMapOnLoad = () => {
        const context = canvas.current.getContext("2d")
        if(context){
            draw(context)
        }
    }

    function getMarginY(rackDes){
        if (rackDes % 2 !== 0 && rackDes !== 7 && rackDes !== 9) {
            return (rackDes - 1) * droneRef.current.height + margin + 40 * Math.trunc(rackDes / 2)
        }
        else if(rackDes === 6 || rackDes === 8){
            return (rackDes - 1) * droneRef.current.height + margin + 40 * Math.trunc((rackDes-1) / 2) + 20
        }
        else if(rackDes === 7 || rackDes === 9){
            return (rackDes - 1) * droneRef.current.height + margin + 40 * Math.trunc(rackDes / 2) + 10
        }
        else{
            return (rackDes - 1) * droneRef.current.height + margin + 40 * Math.trunc((rackDes-1) / 2) + 10
        }
    }


    useEffect(()=>{
        const context = canvas.current.getContext("2d")
        const desX = (17 - rowDes -1) * 55 + 40
        const desY = getMarginY(rackDes)
        if(context){
            if(available || available === undefined){
                context.beginPath() 
                context.drawImage(droneRef.current,droneX,droneY,droneRef.current.width,droneRef.current.height)
            }
            else{
                let tmpX = 890
                while(desX < tmpX){
                    tmpX -= 1
                    context.beginPath()
                    context.drawImage(droneRef.current,tmpX,desY,droneRef.current.width,droneRef.current.height)
                }
            }
        }
    },[rowDes,rackDes,available])

    const handleMapOnClick = (e) =>{
        const rect = canvas.current.getBoundingClientRect()
        const x = e.clientX - rect.left
        const y = e.clientY - rect.top
        const a = 0
        const b = 10
        const row = 16-Math.round((x-40)/55)
        const rack = Math.round((y - 25 -20 * a + b)/45)
        setRackClick(rack)
        setRowClick(row)
    }

    return (
        <div className="mapping">
            <canvas ref={canvas} width={canvasWidth} height={canvasHeight} onClick={handleMapOnClick}/>
            <img ref={imgRef} src="mapping.png" alt="mapping" className="mapping-img" onLoad={handleMapOnLoad} />
            <img ref={droneRef} src="drone.png" alt="drone" className="drone-piece"/>
        </div>        
    )
}

