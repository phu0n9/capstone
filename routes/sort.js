const router = require('express').Router()
const Inventory = require('../model/inventory.model')

router.route('').get(async (req,res) => {
    await Inventory.find({'location':req.query.location})
    .exec((err, inventory) => {
        if (err) return next(err)
        res.send(inventory) // return data JSON
    })
})

router.route('').get((req,res) => {
    Inventory.find({'userId':req.query.userId})
    .then(userId => res.json(userId))
    .catch(err => res.status(400).json('Error: '+err))
})

router.route('').get((req,res) => {
    Inventory.find({'createdAt':
        {
            "$gte": new Date(req.query.time)
        }
    })
    .then(time => {
        res.json(time)
    })
    .catch(err => res.status(400).json('Error: '+err));
})

module.exports = router