const {Schema,model} = require('mongoose')

const History = new Schema({
    _id: String,
    userId: String,
    inventoryId: String
},{   
    timestamps: true,
})

module.exports = model("History",History)