import React from 'react'

export default function PhotoDisplay({clickPhoto}) {
    return (
        <div className="photoDisplay-wrapper">
            {/* Inventory photo */}
            <img src={clickPhoto} alt="inventoryPhoto"/>
        </div>
    )
}
