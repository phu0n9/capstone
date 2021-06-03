const {Schema,model} = require('mongoose')

const User = new Schema({
    _id:String,
    searchData: String,
    inventoryId: [{type: Schema.Types.ObjectId, ref:'Inventory' }]
},{   
    timestamps: true,
})

module.exports = model("User",User)