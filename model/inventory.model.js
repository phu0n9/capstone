const {Schema,model} = require('mongoose')

const Inventory = new Schema({
    data:Object,
    userId:String
},{   
    timestamps: true,
})

module.exports = model("Inventory",Inventory)