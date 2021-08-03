const router = require('express').Router()
const Queue = require('../model/queue.model')
const Pusher = require('pusher')
require('dotenv').config()

const pusher = process.env.PUSHER

router.route('').get(async (req,res) => {
    await Queue.countDocuments(function(err,count){
        if(!err && count !== 0){
            Queue.find().sort({ 'createdAt': -1 })
            .exec((err, item) => {
                if (err) return (err)
                res.send(item)
            })
        }
    })
})

router.route('/delete/:id').delete(async (req, res) => {
    await Queue.findByIdAndDelete(req.params.id).exec((err, inventory) => {
        if (err) return (err)
        const response = {
            message: "Todo successfully deleted",
            id: req.params.id
        }
        return res.status(200).send(response)
    })
})

router.route('/execute/:id').get(async (req, res) => {
    await pusher.trigger('search','keyword',req.params.id)
    .then(() => res.status(200).send("sent request"))
    .catch((error)=>{console.log(error)})
})

module.exports = router