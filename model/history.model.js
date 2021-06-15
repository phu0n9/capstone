const {Schema,model} = require('mongoose')

const History = new Schema({
    userId: String,
    inventoryId: String
},{   
    timestamps: true,
})

module.exports = model("History",History)