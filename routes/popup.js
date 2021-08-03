const router = require('express').Router()
const Queue = require('../model/queue.model')
const Pusher = require('pusher')

const pusher = new Pusher({
    appId: "1219725",
    key: "2ccb32686bdc0f96f50a",
    secret: "84a8c245f48597b3e0bb",
    cluster: "ap1",
    useTLS: true
})

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


router.route('/execute/:id').get((req, res) => {
    pusher.trigger('search','keyword',req.params.id)
})

module.exports = router